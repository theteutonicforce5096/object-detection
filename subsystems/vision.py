from math import hypot
from typing import Callable, Final

from commands2 import Subsystem
from phoenix6 import swerve, utils
from photonlibpy import EstimatedRobotPose, PhotonCamera, PhotonPoseEstimator
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from wpilib import Field2d, SmartDashboard
from wpimath.geometry import Pose2d, Pose3d, Rotation3d, Transform3d, Translation3d
from wpimath.units import inchesToMeters


class Vision(Subsystem):
    """
    Feeds AprilTag pose estimates from the PhotonVision cameras into drivetrain odometry.

    Each camera is polled every loop and its multi-tag solution is checked against the field
    bounds and the robot's motion before it is accepted. Accepted estimates are handed to
    odometry with standard deviations that grow with tag distance and robot speed, so a shaky
    estimate pulls the fused pose less than a confident one.
    """

    # At 30 fps against the 50 Hz robot loop there is normally at most one unread frame per
    # camera, but a loop hiccup or brownout can leave several queued. The roboRIO 2.0 cannot
    # afford unbounded pose solves inside one 20 ms loop, so only the newest few frames are
    # processed and anything older is dropped as stale.
    MAX_RESULTS_PER_CAMERA_PER_LOOP: Final[int] = 5

    def __init__(
        self,
        add_vision_measurement,
        get_current_swerve_state: Callable[[], swerve.SwerveDrivetrain.SwerveDriveState],
        get_robot_tilt: Callable[[], tuple[float, float]],
        set_camera_pose: Callable[[str, Pose2d], None],
        field_type: str,
        linear_std_dev_baseline: float,
        angular_std_dev_baseline: float,
        camera_std_dev_factors: tuple[float, ...],
        max_linear_speed: float,
        max_angular_speed: float,
        max_tilt_deg: float,
    ):
        """
        Construct the vision subsystem and per-camera pose estimators.

        :param add_vision_measurement: Callback used to inject accepted vision measurements into
            drivetrain odometry.
        :type add_vision_measurement:
            Callable[[wpimath.geometry.Pose2d, float, tuple[float, float, float]], None]
        :param get_current_swerve_state: Function that returns the current drivetrain state.
        :type get_current_swerve_state:
            Callable[[], phoenix6.swerve.SwerveDrivetrain.SwerveDriveState]
        :param get_robot_tilt: Function that returns the current robot pitch and roll in degrees.
        :type get_robot_tilt: Callable[[], tuple[float, float]]
        :param set_camera_pose: Callback used to show a camera's latest accepted estimate on the
            drivetrain's field widget.
        :type set_camera_pose: Callable[[str, wpimath.geometry.Pose2d], None]
        :param field_type: Name of the active field variant used to load the AprilTag layout.
        :type field_type: str
        :param linear_std_dev_baseline: Baseline linear measurement standard deviation in meters.
        :type linear_std_dev_baseline: float
        :param angular_std_dev_baseline: Baseline angular measurement standard deviation in radians.
        :type angular_std_dev_baseline: float
        :param camera_std_dev_factors: Per-camera multipliers applied to the baseline standard
            deviations.
        :type camera_std_dev_factors: tuple[float, ...]
        :param max_linear_speed: Linear speed used both as a hard rejection gate on measurements
            and as the reference speed for trust scaling. VisionConstants supplies 25% of the
            drivetrain's physical maximum.
        :type max_linear_speed: float
        :param max_angular_speed: Angular speed used both as a hard rejection gate on
            measurements and as the reference speed for trust scaling. VisionConstants supplies
            20% of the drivetrain's physical maximum.
        :type max_angular_speed: float
        :param max_tilt_deg: Tilt threshold in degrees beyond which measurements are rejected.
        :type max_tilt_deg: float
        """
        Subsystem.__init__(self)

        self.add_vision_measurement = add_vision_measurement
        self.get_current_swerve_state = get_current_swerve_state
        self.get_robot_tilt = get_robot_tilt
        self.set_camera_pose = set_camera_pose
        self.field_type = field_type
        self.april_tag_layout = self._load_april_tag_layout(field_type)
        # Field bounds never change, so read them once instead of every rejection check.
        self.field_length = self.april_tag_layout.getFieldLength()
        self.field_width = self.april_tag_layout.getFieldWidth()
        self.linear_std_dev_baseline = linear_std_dev_baseline
        self.angular_std_dev_baseline = angular_std_dev_baseline
        self.camera_std_dev_factors = camera_std_dev_factors
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.max_tilt_deg = max_tilt_deg

        # Each camera's name and where it sits on the robot. The order here is the camera index
        # used by camera_std_dev_factors, so adding a camera means adding a factor to match.
        camera_layout = [
            (
                "back_camera",
                Transform3d(
                    Translation3d(inchesToMeters(4.25), inchesToMeters(10), inchesToMeters(20.5)),
                    Rotation3d.fromDegrees(1, 0, 180),
                ),
            ),
            (
                "front_left_camera",
                Transform3d(
                    Translation3d(
                        inchesToMeters(13.5), inchesToMeters(12.5), inchesToMeters(20.25)
                    ),
                    Rotation3d.fromDegrees(-1, 0, 0),
                ),
            ),
            (
                "front_right_camera",
                Transform3d(
                    Translation3d(
                        inchesToMeters(13.5), inchesToMeters(-12.25), inchesToMeters(20.25)
                    ),
                    Rotation3d.fromDegrees(-4, 0, 0),
                ),
            ),
        ]

        self.cameras = []
        self.camera_pose_fields = {}
        for name, robot_to_camera in camera_layout:
            self.cameras.append(
                (
                    name,
                    PhotonCamera(name),
                    PhotonPoseEstimator(self.april_tag_layout, robot_to_camera),
                )
            )

            # Each camera gets a field widget of its own. The shared drivetrain field draws all
            # of its extra poses the same way, so a separate field is what gives a camera its own
            # color and title in Elastic.
            camera_field = Field2d()
            SmartDashboard.putData(f"Vision/{name} Pose", camera_field)
            self.camera_pose_fields[name] = camera_field

    def _load_april_tag_layout(self, field_type: str) -> AprilTagFieldLayout:
        """
        Load the AprilTag layout matching the configured field variant.

        :param field_type: Name of the active field variant.
        :type field_type: str
        :returns: AprilTag field layout for the requested field variant.
        :rtype: robotpy_apriltag.AprilTagFieldLayout
        """
        field_layouts = {
            "AndyMark": AprilTagField.k2026RebuiltAndyMark,
            "Welded": AprilTagField.k2026RebuiltWelded,
        }
        april_tag_field = field_layouts.get(field_type)
        if april_tag_field is None:
            raise ValueError(f"Unsupported field type: {field_type}")

        return AprilTagFieldLayout.loadField(april_tag_field)

    def periodic(self):
        """Poll each camera and push its accepted pose measurements into odometry."""
        current_state = self.get_current_swerve_state()
        accepted_estimates = 0

        for camera_index, (name, camera, pose_est) in enumerate(self.cameras):
            measurements = self.get_vision_measurements(
                camera_index, name, camera, pose_est, current_state
            )

            for pose, timestamp, std_devs in measurements:
                self.add_vision_measurement(pose, timestamp, std_devs)

            accepted_estimates += len(measurements)

        SmartDashboard.putNumber("Vision/Accepted Estimates", accepted_estimates)

    def _publish_camera_status(
        self,
        camera_name: str,
        has_estimate: bool,
        tag_count: int,
        avg_tag_distance: float,
        linear_std_dev: float,
    ):
        """
        Publish one camera's status for this loop to the dashboard.

        Unknown numbers are published as -1 because NetworkTables has no way to say "no value".

        :param camera_name: Name of the camera being reported.
        :type camera_name: str
        :param has_estimate: Whether the camera produced an accepted estimate this loop.
        :type has_estimate: bool
        :param tag_count: Number of tags the camera saw.
        :type tag_count: int
        :param avg_tag_distance: Average robot-to-tag distance in meters, or -1 when unknown.
        :type avg_tag_distance: float
        :param linear_std_dev: Linear standard deviation handed to odometry, or -1 when unknown.
        :type linear_std_dev: float
        """
        SmartDashboard.putBoolean(f"Vision/{camera_name}/Has Estimate", has_estimate)
        SmartDashboard.putNumber(f"Vision/{camera_name}/Tag Count", tag_count)
        SmartDashboard.putNumber(f"Vision/{camera_name}/Avg Tag Distance (m)", avg_tag_distance)
        SmartDashboard.putNumber(f"Vision/{camera_name}/Std Dev (m)", linear_std_dev)

    def reject_pose_estimate(
        self, pose: Pose3d, current_state: swerve.SwerveDrivetrain.SwerveDriveState
    ) -> bool:
        """
        Check a pose estimate against the field bounds and the robot's current motion.

        A solve that puts the robot off the field or underground is wrong outright. One taken
        while the robot is moving or tilted fast enough is unreliable, since the image and the
        odometry sample it is matched against no longer describe the same instant.

        :param pose: Estimated robot pose from PhotonVision.
        :type pose: wpimath.geometry.Pose3d
        :param current_state: Current swerve drivetrain state used for motion gating.
        :type current_state: phoenix6.swerve.SwerveDrivetrain.SwerveDriveState
        :returns: True when the pose estimate should be rejected.
        :rtype: bool
        """
        current_vx = abs(current_state.speeds.vx)
        current_vy = abs(current_state.speeds.vy)
        current_omega = abs(current_state.speeds.omega)
        pitch_deg, roll_deg = self.get_robot_tilt()

        return not (
            -0.25 < pose.Z() < 0.5
            and 0.0 < pose.X() < self.field_length
            and 0.0 < pose.Y() < self.field_width
            and current_vx <= self.max_linear_speed
            and current_vy <= self.max_linear_speed
            and current_omega <= self.max_angular_speed
            and abs(pitch_deg) <= self.max_tilt_deg
            and abs(roll_deg) <= self.max_tilt_deg
        )

    def get_average_tag_distance(self, estimated_pose: EstimatedRobotPose) -> float | None:
        """
        Compute the mean robot-to-tag distance for the tags used in a solve.

        :param estimated_pose: Multi-tag pose estimate returned by PhotonVision.
        :type estimated_pose: photonlibpy.EstimatedRobotPose
        :returns: Average robot-to-tag distance in meters, or None when no valid tags are present.
        :rtype: float | None
        """
        robot_translation = estimated_pose.estimatedPose.toPose2d().translation()

        total_distance = 0.0
        tag_count = 0

        for target in estimated_pose.targetsUsed:
            tag_pose = self.april_tag_layout.getTagPose(target.getFiducialId())
            if tag_pose is None:
                continue

            total_distance += robot_translation.distance(tag_pose.toPose2d().translation())
            tag_count += 1

        if tag_count == 0:
            return None

        return total_distance / tag_count

    def speed_factor(self, current_speed: float, max_speed: float) -> float:
        """
        Grow the measurement uncertainty once the robot passes half of the speed reference.

        Below that point the estimate is taken at face value. Above it the standard deviation
        scales with speed, since motion blur and pose latency both get worse the faster the
        robot is moving.

        :param current_speed: Current robot speed for the axis being evaluated.
        :type current_speed: float
        :param max_speed: Maximum speed used as the scaling reference.
        :type max_speed: float
        :returns: Multiplier applied to the baseline standard deviation.
        :rtype: float
        """
        # A non-positive reference speed cannot produce a meaningful ratio, so the estimate is
        # taken at face value instead of dividing by zero.
        if max_speed <= 0.0:
            return 1.0

        speed_ratio = current_speed / max_speed
        if speed_ratio <= 0.5:
            return 1.0
        else:
            return speed_ratio / 0.5

    def calc_std_dev(
        self,
        estimated_pose: EstimatedRobotPose,
        current_state: swerve.SwerveDrivetrain.SwerveDriveState,
        camera_index: int,
        avg_tag_distance: float | None,
    ):
        """
        Estimate how much to trust a camera pose solution.

        Distant tags and single-tag solves are less precise, and a fast-moving robot makes any
        solve less reliable, so the baseline deviation is scaled by all three. Odometry weighs
        the measurement against its own estimate using the numbers returned here.

        :param estimated_pose: Pose estimate returned by PhotonVision.
        :type estimated_pose: photonlibpy.EstimatedRobotPose
        :param current_state: Current drivetrain state used to scale measurement trust.
        :type current_state: phoenix6.swerve.SwerveDrivetrain.SwerveDriveState
        :param camera_index: Index of the camera that produced the estimate.
        :type camera_index: int
        :param avg_tag_distance: Average robot-to-tag distance in meters, or None when no valid
            tags are present.
        :type avg_tag_distance: float | None
        :returns: Standard deviation tuple for x, y, and heading, or None when the estimate is
            unusable.
        :rtype: tuple[float, float, float] | None
        """
        tag_count = len(estimated_pose.targetsUsed)
        if avg_tag_distance is None or tag_count == 0:
            return None

        current_linear_speed = hypot(current_state.speeds.vx, current_state.speeds.vy)

        # Uncertainty grows with the square of tag distance and shrinks as more tags contribute
        # to the solve.
        distance_factor = (avg_tag_distance**2.0) / tag_count

        linear_speed_factor = self.speed_factor(current_linear_speed, self.max_linear_speed)

        camera_factor = (
            self.camera_std_dev_factors[camera_index]
            if camera_index < len(self.camera_std_dev_factors)
            else 1.0
        )

        linear_std_dev = (
            self.linear_std_dev_baseline * distance_factor * camera_factor * linear_speed_factor
        )

        # The Pigeon tracks heading far better than vision does, so the angular baseline is set
        # high enough that odometry effectively ignores the vision heading. Scaling it would
        # make no practical difference.
        angular_std_dev = self.angular_std_dev_baseline

        return (linear_std_dev, linear_std_dev, angular_std_dev)

    def get_vision_measurements(
        self,
        camera_index: int,
        camera_name: str,
        camera: PhotonCamera,
        pose_est: PhotonPoseEstimator,
        current_state: swerve.SwerveDrivetrain.SwerveDriveState,
    ) -> list[tuple[Pose2d, float, tuple[float, float, float]]]:
        """
        Return every accepted pose measurement from one camera since the last loop.

        Results are processed oldest to newest so odometry receives measurements in time order,
        capped at MAX_RESULTS_PER_CAMERA_PER_LOOP with the oldest frames dropped first, since
        stale frames are the least valuable. The dashboard status reflects the newest result's
        outcome.

        :param camera_index: Index of the camera in the subsystem camera list.
        :type camera_index: int
        :param camera_name: Name of the camera, used for its dashboard keys.
        :type camera_name: str
        :param camera: PhotonCamera instance to read from.
        :type camera: photonlibpy.PhotonCamera
        :param pose_est: PhotonPoseEstimator associated with the camera.
        :type pose_est: photonlibpy.PhotonPoseEstimator
        :param current_state: Current drivetrain state used for gating and covariance scaling.
        :type current_state: phoenix6.swerve.SwerveDrivetrain.SwerveDriveState
        :returns: Accepted pose measurement tuples, oldest first; empty when no usable result
            exists.
        :rtype: list[tuple[wpimath.geometry.Pose2d, float, tuple[float, float, float]]]
        """
        # Status is published on every path through this method, not just the accepting one.
        # From the dashboard, an unplugged camera, one that sees no tags, and one whose solve was
        # rejected otherwise look identical.
        SmartDashboard.putBoolean(f"Vision/{camera_name}/Connected", camera.isConnected())

        results = camera.getAllUnreadResults()
        if len(results) == 0:
            self._publish_camera_status(camera_name, False, 0, -1.0, -1.0)
            return []

        measurements = []
        newest_index = len(results) - 1
        start_index = max(0, len(results) - self.MAX_RESULTS_PER_CAMERA_PER_LOOP)

        for result_index in range(start_index, len(results)):
            result = results[result_index]
            is_newest = result_index == newest_index
            tag_count = len(result.getTargets())

            # A single tag cannot pin down a pose well enough to trust, so only multi-tag solves
            # are used.
            if tag_count <= 1:
                if is_newest:
                    self._publish_camera_status(camera_name, False, tag_count, -1.0, -1.0)
                continue

            pose = pose_est.estimateCoprocMultiTagPose(result)

            if pose is None or self.reject_pose_estimate(pose.estimatedPose, current_state):
                if is_newest:
                    self._publish_camera_status(camera_name, False, tag_count, -1.0, -1.0)
                continue

            avg_tag_distance = self.get_average_tag_distance(pose)
            std_devs = self.calc_std_dev(pose, current_state, camera_index, avg_tag_distance)

            # A solve with no usable covariance cannot be weighed against odometry, so it is
            # dropped instead of forwarded.
            if std_devs is None:
                if is_newest:
                    self._publish_camera_status(
                        camera_name,
                        False,
                        tag_count,
                        avg_tag_distance if avg_tag_distance is not None else -1.0,
                        -1.0,
                    )
                continue

            if is_newest:
                self._publish_camera_status(
                    camera_name,
                    True,
                    tag_count,
                    avg_tag_distance if avg_tag_distance is not None else -1.0,
                    std_devs[0],
                )

            # PhotonVision stamps the pose in the FPGA/NT time base, while CTRE's odometry
            # expects the Phoenix time base. Converting is what lands the sample in the right
            # slot of the odometry buffer instead of roughly 1.1 million seconds in the past.
            measurements.append(
                (
                    pose.estimatedPose.toPose2d(),
                    utils.fpga_to_current_time(pose.timestampSeconds),
                    std_devs,
                )
            )

        if len(measurements) > 0:
            # The widget holds the last accepted pose after a camera goes quiet, which shows
            # where it last had a fix. The Has Estimate key is what says whether the pose is
            # still current.
            last_accepted_pose = measurements[-1][0]
            self.camera_pose_fields[camera_name].setRobotPose(last_accepted_pose)
            self.set_camera_pose(camera_name, last_accepted_pose)

        return measurements
