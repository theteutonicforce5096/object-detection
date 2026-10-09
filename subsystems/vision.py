from math import hypot
from typing import Callable, Final, Literal

from commands2 import Subsystem, PrintCommand
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
                    robot_to_camera,
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
        current_pose = self.get_current_swerve_state().pose

        for camera_index, (name, camera, robot_to_cam) in enumerate(self.cameras):
            detected = self.detect_object(camera, current_pose, robot_to_cam)
            if detected:
                PrintCommand(f"{name} detected fuel!").schedule()
            
    def detect_object(self, 
                      camera: PhotonCamera, 
                      current_pose: Pose2d, 
                      robot_to_cam: Translation3d
    ) -> bool:
        results = camera.getAllUnreadResults()
        
        if len(results) == 0:
            return False
            
        latest_result = results[-1]
        if (fuel := latest_result.getBestTarget()) is not None and fuel.getFiducialId() == 0:
            # Figure math to go from box to pose of ball using robot pose as base
            fuel_yaw = fuel.getYaw()
            fuel_pitch = fuel.getPitch()
            
            return True
        else:
            return False
        