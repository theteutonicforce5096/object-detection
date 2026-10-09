from math import copysign, hypot, pi
from typing import Any, Callable, cast

from commands2 import FunctionalCommand, Subsystem
from commands2.sysid import SysIdRoutine
from pathplannerlib.auto import AutoBuilder, RobotConfig
from pathplannerlib.controller import PIDConstants, PPHolonomicDriveController
from phoenix6 import SignalLogger, hardware, swerve, utils
from wpilib import (
    DriverStation,
    Field2d,
    Notifier,
    RobotBase,
    RobotController,
    SmartDashboard,
    Timer,
    reportError,
    reportWarning,
)
from wpilib.sysid import SysIdRoutineLog
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds, SwerveDrive4Kinematics, SwerveDrive4Odometry

from constants.shot_calculator_constants import (
    default_shooter_offset,
    get_hub_center,
    get_hub_reset_pose,
)
from subsystems.device_config import check_signal_status, configure_device


class SwerveDrivetrain(Subsystem, swerve.SwerveDrivetrain):
    """
    Drives the swerve modules and tracks where the robot is on the field.

    Wraps CTRE's swerve API with the pieces this robot needs on top of it: shaped and
    rate-limited teleop input, hub auto-alignment about the shooter, a PathPlanner hookup for
    autonomous, and a plain odometry track kept next to the vision-fused pose for comparison.
    """

    def __init__(
        self,
        drive_motor_type: Any,
        steer_motor_type: Any,
        encoder_type: Any,
        drivetrain_constants: swerve.SwerveDrivetrainConstants,
        modules: list[swerve.SwerveModuleConstants],
        num_config_attempts: int,
        odometry_update_frequency: float,
        max_linear_speed: float,
        max_angular_speed: float,
        max_linear_rate_of_change: float,
        max_angular_rate_of_change: float,
        teleop_precision_scale: float,
        teleop_default_scale: float,
        teleop_turbo_scale: float,
        teleop_rotation_scale: float,
        teleop_translation_deadband: float,
        teleop_rotation_deadband: float,
        field_type: str,
        wheel_radius: float,
    ):
        """
        Initialize the swerve drivetrain using the specified constants.

        :param drive_motor_type: Type of the drive motor
        :type drive_motor_type: Any
        :param steer_motor_type: Type of the steer motor
        :type steer_motor_type: Any
        :param encoder_type: Type of the azimuth encoder
        :type encoder_type: Any
        :param drivetrain_constants: Drivetrain-wide constants for the swerve drive
        :type drivetrain_constants: phoenix6.swerve.SwerveDrivetrainConstants
        :param modules: Constants for each specific module
        :type modules: list[phoenix6.swerve.SwerveModuleConstants]
        :param num_config_attempts: Number of times to attempt to configure the license probe
            device before the swerve devices are constructed.
        :type num_config_attempts: int
        :param odometry_update_frequency: The frequency to run the odometry loop at.
        :type odometry_update_frequency: hertz
        :param max_linear_speed: Max linear speed of drivetrain in meters per second.
        :type max_linear_speed: float
        :param max_angular_speed: Max angular velocity of drivetrain in radians per second.
        :type max_angular_speed: float
        :param max_linear_rate_of_change: Max linear rate of change of drivetrain in meters per
            second per 20 ms loop.
        :type max_linear_rate_of_change: float
        :param max_angular_rate_of_change: Max angular rate of change of drivetrain in radians
            per second per 20 ms loop.
        :type max_angular_rate_of_change: float
        :param teleop_precision_scale: Fraction of max speed applied when the left trigger is held.
        :type teleop_precision_scale: float
        :param teleop_default_scale: Fraction of max speed applied when no trigger is held.
        :type teleop_default_scale: float
        :param teleop_turbo_scale: Fraction of max speed applied when both triggers are held.
        :type teleop_turbo_scale: float
        :param teleop_rotation_scale: Fraction of max angular speed applied to teleop rotation.
        :type teleop_rotation_scale: float
        :param teleop_translation_deadband: Translation stick deadband as a fraction of full travel.
        :type teleop_translation_deadband: float
        :param teleop_rotation_deadband: Rotation stick deadband as a fraction of full travel.
        :type teleop_rotation_deadband: float
        :param field_type: Current field variant used for field geometry.
        :type field_type: str
        :param wheel_radius: Effective drivetrain wheel radius in meters.
        :type wheel_radius: float
        """
        # Initialize parent classes
        Subsystem.__init__(self)

        # CTRE's swerve constructor applies module configs internally with only two quick
        # attempts, which loses the race against the Phoenix license checks that reject configs
        # on a pro bus for the first few seconds after code start. Configuring the Pigeon
        # through the robust retry path first proves the bus and license checks are live before
        # those fragile applies run; the constructor then reapplies the Pigeon configs, which
        # is harmless.
        if not RobotBase.isSimulation():
            license_probe = hardware.Pigeon2(
                drivetrain_constants.pigeon2_id, drivetrain_constants.can_bus_name
            )
            configure_device(
                license_probe, drivetrain_constants.pigeon2_configs, num_config_attempts
            )

        swerve.SwerveDrivetrain.__init__(
            self,
            drive_motor_type,
            steer_motor_type,
            encoder_type,
            drivetrain_constants,
            odometry_update_frequency,
            modules,
        )

        if RobotBase.isSimulation():
            self._start_sim_thread()
        else:
            for num in range(4):
                module = self.get_module(num)
                check_signal_status(
                    module.drive_motor.optimize_bus_utilization(),
                    f"Drivetrain module {num} drive motor bus optimization",
                )
                check_signal_status(
                    module.steer_motor.optimize_bus_utilization(),
                    f"Drivetrain module {num} steer motor bus optimization",
                )
                check_signal_status(
                    module.encoder.optimize_bus_utilization(),
                    f"Drivetrain module {num} encoder bus optimization",
                )

            check_signal_status(
                self.pigeon2.optimize_bus_utilization(), "Pigeon 2 bus optimization"
            )
            # Only the Pigeon signals something reads stay at a high rate: pitch and roll gate
            # vision measurements, and yaw drives wheel-radius characterization. 100 Hz is plenty
            # for the 20 ms loop while everything else stays trimmed.
            check_signal_status(
                self.pigeon2.get_pitch().set_update_frequency(100.0),
                "Pigeon 2 pitch update frequency",
            )
            check_signal_status(
                self.pigeon2.get_roll().set_update_frequency(100.0),
                "Pigeon 2 roll update frequency",
            )
            check_signal_status(
                self.pigeon2.get_yaw().set_update_frequency(100.0),
                "Pigeon 2 yaw update frequency",
            )

        # Create max speed and max acceleration variables
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.max_linear_rate_of_change = max_linear_rate_of_change
        self.max_angular_rate_of_change = max_angular_rate_of_change

        # Create teleop input shaping variables
        self.teleop_precision_scale = teleop_precision_scale
        self.teleop_default_scale = teleop_default_scale
        self.teleop_turbo_scale = teleop_turbo_scale
        self.teleop_rotation_scale = teleop_rotation_scale
        self.teleop_translation_deadband = teleop_translation_deadband
        self.teleop_rotation_deadband = teleop_rotation_deadband

        # Last operator-perspective command, kept so teleop input can be slew-rate limited
        self._last_commanded_vx = 0.0
        self._last_commanded_vy = 0.0
        self._last_commanded_omega = 0.0

        # Create field type variable
        self.field_type = field_type

        # Create wheel and drive base radius variables for effective wheel radius
        # characterization command
        self.wheel_radius = wheel_radius
        self._drive_base_radius = sum(
            module_location.norm() for module_location in self.module_locations
        ) / len(self.module_locations)

        # Create shooter position variable
        self.shooter_offset = default_shooter_offset

        # Create current alliance variable
        self.current_alliance = None

        # Standard teleop driving request. Velocities are given from the driver's point of view,
        # so "forward" always means away from the driver station regardless of robot heading.
        # https://www.chiefdelphi.com/t/motion-magic-velocity-control-for-drive-motors-in-phoenix6-swerve-drive-api/483669/6
        self.field_centric_request = (
            swerve.requests.FieldCentric()
            .with_forward_perspective(swerve.requests.ForwardPerspectiveValue.OPERATOR_PERSPECTIVE)
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
            .with_steer_request_type(swerve.SwerveModule.SteerRequestType.MOTION_MAGIC_EXPO)
            .with_desaturate_wheel_speeds(True)
        )

        # Create Apply Robot Speeds Request for PathPlanner
        self._apply_robot_speeds = (
            swerve.requests.ApplyRobotSpeeds()
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
            .with_steer_request_type(swerve.SwerveModule.SteerRequestType.MOTION_MAGIC_EXPO)
            .with_desaturate_wheel_speeds(True)
        )

        # Request used by the hub auto-align helpers. Its target angle is a field-frame
        # (blue-origin) heading, so it takes the blue-alliance perspective instead of the
        # operator perspective the driving request uses.
        self.rotate_robot_request = (
            swerve.requests.FieldCentricFacingAngle()
            .with_forward_perspective(swerve.requests.ForwardPerspectiveValue.BLUE_ALLIANCE)
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
            .with_steer_request_type(swerve.SwerveModule.SteerRequestType.POSITION)
            .with_desaturate_wheel_speeds(True)
            .with_heading_pid(5.0, 0, 0)
        )

        self.rotate_robot_pid_controller = self.rotate_robot_request.heading_controller
        self.rotate_robot_pid_controller.setTolerance(Rotation2d.fromDegrees(1.0).radians())

        self.set_forward_perspective()
        self.reset_teleop_drive_state()

        self._configure_auto_builder()

        # Swerve requests for SysId characterization
        self.translation_characterization = swerve.requests.SysIdSwerveTranslation()
        self.steer_characterization = swerve.requests.SysIdSwerveSteerGains()
        self.rotation_characterization = swerve.requests.SysIdSwerveRotation()

        # Create SysId routine for characterizing drive.
        def record_translation_state(state):
            SignalLogger.write_string(
                "SysId_Translation_State", SysIdRoutineLog.stateEnumToString(state)
            )

        # The mechanism log callback is a no-op in all three routines because Phoenix already
        # logs every signal SysId needs through SignalLogger.
        self.sys_id_routine_translation = SysIdRoutine(
            SysIdRoutine.Config(
                rampRate=1.0,
                stepVoltage=5.0,
                timeout=5.0,
                recordState=record_translation_state,
            ),
            SysIdRoutine.Mechanism(
                lambda output: self.set_control(
                    self.translation_characterization.with_volts(output)
                ),
                lambda log: None,
                self,
            ),
        )

        # Create SysId routine for characterizing steer.
        def record_steer_state(state):
            SignalLogger.write_string("SysId_Steer_State", SysIdRoutineLog.stateEnumToString(state))

        self.sys_id_routine_steer = SysIdRoutine(
            SysIdRoutine.Config(
                rampRate=1.0,
                stepVoltage=4.0,
                timeout=5.0,
                recordState=record_steer_state,
            ),
            SysIdRoutine.Mechanism(
                lambda output: self.set_control(self.steer_characterization.with_volts(output)),
                lambda log: None,
                self,
            ),
        )

        def record_rotation_state(state):
            SignalLogger.write_string(
                "SysId_Rotation_State", SysIdRoutineLog.stateEnumToString(state)
            )

        self.sys_id_routine_rotation = SysIdRoutine(
            SysIdRoutine.Config(
                rampRate=pi / 6,
                stepVoltage=4.0,
                timeout=5.0,
                recordState=record_rotation_state,
            ),
            SysIdRoutine.Mechanism(
                lambda output: self.set_control(
                    self.rotation_characterization.with_rotational_rate(output)
                ),
                lambda log: None,
                self,
            ),
        )

        # Routines this subsystem can characterize. RobotContainer gathers these, along with the
        # other subsystems' routines, into the single SysId chooser on the dashboard.
        self.sys_id_routines = [
            ("Translation", self.sys_id_routine_translation),
            ("Steer", self.sys_id_routine_steer),
            ("Rotation", self.sys_id_routine_rotation),
        ]

        # One field widget holding every pose the robot believes in. The fused pose is the robot
        # marker, and the odometry-only track plus each camera's estimate ride along as field
        # objects, which makes it easy to see whether they agree.
        self.robot_pose_field = Field2d()
        SmartDashboard.putData("Robot Pose", self.robot_pose_field)

        # Plain dead reckoning running beside the fused pose with no vision mixed in. The gap
        # between the two shows how hard vision is correcting. It is seeded from raw_heading
        # because that is the one heading vision never touches.
        current_state = self.get_state()
        self.odometry_only = SwerveDrive4Odometry(
            # Phoenix types the shared kinematics as a union over module counts, but this
            # drivetrain always has four modules.
            cast(SwerveDrive4Kinematics, self.kinematics),
            current_state.raw_heading,
            self._get_module_positions(current_state),
            current_state.pose,
        )
        self.odometry_only_pose = current_state.pose

        def update_odometry_only(state):
            """
            Integrate the odometry-only pose from the latest swerve state.

            :param state: Swerve drive state supplied by the odometry thread.
            :type state: phoenix6.swerve.SwerveDrivetrain.SwerveDriveState
            """
            self.odometry_only_pose = self.odometry_only.update(
                state.raw_heading, self._get_module_positions(state)
            )

        # Phoenix calls this on its odometry thread on every update, so both tracks integrate
        # just as often. The callback only integrates, since slow work here would stall that
        # thread; publishing happens in periodic() instead.
        self.register_telemetry(update_odometry_only)

    def _configure_auto_builder(self):
        """Configure PathPlanner to drive this drivetrain during autonomous paths."""
        try:
            config = RobotConfig.fromGUISettings()
        except Exception:
            reportError("PathPlanner robot config failed to load; autonomous paths disabled", True)
            return

        AutoBuilder.configure(
            lambda: self.get_state().pose,  # Supplier of current robot pose
            self.reset_all_poses,  # Consumer for seeding pose against auto
            lambda: self.get_state().speeds,  # Supplier of current robot speeds
            # Consumer of ChassisSpeeds and feedforwards to drive the robot
            lambda speeds, feedforwards: self.set_control(
                self._apply_robot_speeds.with_speeds(ChassisSpeeds.discretize(speeds, 0.020))
                .with_wheel_force_feedforwards_x(feedforwards.robotRelativeForcesXNewtons)
                .with_wheel_force_feedforwards_y(feedforwards.robotRelativeForcesYNewtons)
            ),
            PPHolonomicDriveController(
                # PID constants for translation
                PIDConstants(10.0, 0.0, 0.0),
                # PID constants for rotation
                PIDConstants(7.0, 0.0, 0.0),
            ),
            config,
            # Paths are drawn on the blue side of the field, so they are mirrored when the robot
            # is on the red alliance.
            lambda: (
                (DriverStation.getAlliance() or DriverStation.Alliance.kBlue)
                == DriverStation.Alliance.kRed
            ),
            self,  # Subsystem for requirements
        )

    def _start_sim_thread(self):
        """Start the notifier that advances the Phoenix swerve simulation."""

        def _sim_periodic():
            """Advance the Phoenix drivetrain simulator by the time since the last call."""
            current_time = utils.get_current_time_seconds()
            delta_time = current_time - self._last_sim_time
            self._last_sim_time = current_time

            self.update_sim_state(delta_time, RobotController.getBatteryVoltage())

        # The 4 ms notifier steps the physics model far more often than the 20 ms robot loop
        # would, which keeps the simulated modules and gyro closer to how the real hardware
        # responds between loops.
        self._last_sim_time = utils.get_current_time_seconds()
        self._sim_notifier = Notifier(_sim_periodic)
        self._sim_notifier.startPeriodic(0.004)  # 4 ms

    def periodic(self):
        """Publish the robot's fused and odometry-only poses to the dashboard."""
        current_state = self.get_state()
        odometry_only_pose = self.odometry_only_pose

        self.robot_pose_field.setRobotPose(current_state.pose)
        self.robot_pose_field.getObject("Odometry").setPose(odometry_only_pose)

        # How far vision has pulled the robot away from raw dead reckoning. A small number means
        # vision is nudging the pose; a large one means it is fighting odometry.
        SmartDashboard.putNumber(
            "Drivetrain/Vision Correction (m)",
            current_state.pose.translation().distance(odometry_only_pose.translation()),
        )

    def set_camera_pose(self, camera_name: str, pose: Pose2d):
        """
        Show a camera's latest accepted pose estimate alongside the robot pose.

        :param camera_name: Name of the camera that produced the estimate.
        :type camera_name: str
        :param pose: Pose the camera estimated.
        :type pose: wpimath.geometry.Pose2d
        """
        self.robot_pose_field.getObject(camera_name).setPose(pose)

    @staticmethod
    def _get_module_positions(state):
        """
        Pack the module positions out of a swerve state into the tuple odometry expects.

        :param state: Swerve drive state to read the module positions from.
        :type state: phoenix6.swerve.SwerveDrivetrain.SwerveDriveState
        :returns: The four module positions in module order.
        :rtype: tuple
        """
        front_left, front_right, back_left, back_right = state.module_positions
        return (front_left, front_right, back_left, back_right)

    def set_forward_perspective(self):
        """
        Point field-oriented "forward" away from the driver station for the current alliance.

        Field coordinates always use the blue-alliance origin, so the two alliances face opposite
        directions on the same field. Setting the perspective is what makes pushing the stick
        forward drive away from the drivers on either side.
        """
        alliance_color = DriverStation.getAlliance()
        if alliance_color == DriverStation.Alliance.kRed:
            # Red drivers look down the field toward the blue wall, which is 180 degrees.
            self.set_operator_perspective_forward(Rotation2d.fromDegrees(180))
            self.current_alliance = DriverStation.Alliance.kRed
        else:
            # Blue drivers look toward the red wall, which is 0 degrees. An undetected alliance
            # falls back to this so the robot still drives sensibly off the field.
            self.set_operator_perspective_forward(Rotation2d.fromDegrees(0))
            self.current_alliance = DriverStation.Alliance.kBlue

    def reset_teleop_drive_state(self):
        """
        Seed the teleop slew-rate state from the robot's current velocity.

        The default drive command is interrupted and rescheduled often, so the slew state is
        seeded from the robot's real velocity in the operator-perspective command frame. Starting
        from zero while the robot is moving would command a hard stop followed by a re-accel.
        """
        current_state = self.get_state()

        # Convert the measured robot-relative velocity into the field frame, then into the
        # operator-perspective command frame used by the field-centric request.
        field_relative_speeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            current_state.speeds, current_state.pose.rotation()
        )
        command_velocity = Translation2d(
            field_relative_speeds.vx, field_relative_speeds.vy
        ).rotateBy(-self.get_operator_forward_direction())

        self._last_commanded_vx = command_velocity.x
        self._last_commanded_vy = command_velocity.y
        self._last_commanded_omega = current_state.speeds.omega

    def reset_all_poses(self, pose: Pose2d):
        """
        Seed both the fused pose and the odometry-only track to the same pose.

        Anything that teleports the robot has to move both trackers. If only the fused pose moved,
        the odometry-only track would keep its old pose and the dashboard comparison between them
        would stop meaning anything.

        :param pose: Pose to seed both trackers to.
        :type pose: wpimath.geometry.Pose2d
        """
        self.reset_pose(pose)
        self.odometry_only.resetPose(pose)
        self.odometry_only_pose = pose

    def reset_pose_hub(self):
        """
        Re-seed odometry from a known spot at the hub.

        Drivers use this after odometry has drifted: park the robot against the hub, press the
        button, and both pose trackers snap back to the surveyed reference pose.
        """
        self.reset_all_poses(get_hub_reset_pose(self.field_type, self.current_alliance))

    def get_robot_tilt(self) -> tuple[float, float]:
        """
        Read the robot's current pitch and roll from the Pigeon 2.

        :returns: Pitch and roll in degrees.
        :rtype: tuple[float, float]
        """
        return (self.pigeon2.get_pitch().value_as_double, self.pigeon2.get_roll().value_as_double)

    def _get_operator_drive_request(
        self,
        left_trigger_pressed: bool,
        right_trigger_pressed: bool,
        forward_speed: float,
        strafe_speed: float,
        rotation_speed: float,
    ):
        """
        Turn raw stick and trigger input into a field-centric drive request.

        The triggers pick a throttle scale, the sticks are deadbanded and curved so small motions
        stay gentle, and the result is slew-rate limited so the drivers cannot ask for more
        acceleration than the robot can deliver without tipping or slipping.

        :param left_trigger_pressed: Whether the left trigger of the operator's controller is
            pressed or not.
        :type left_trigger_pressed: bool
        :param right_trigger_pressed: Whether the right trigger of the operator's controller is
            pressed or not.
        :type right_trigger_pressed: bool
        :param forward_speed: Desired forward speed of the operator in terms of percent of max
            linear speed where forward is positive.
        :type forward_speed: float
        :param strafe_speed: Desired strafe speed of the operator in terms of percent of max
            linear speed where left is positive.
        :type strafe_speed: float
        :param rotation_speed: Desired rotation speed of the operator in terms of percent of max
            angular speed where counterclockwise is positive.
        :type rotation_speed: float
        """
        # Both triggers is turbo, left trigger alone is precision, anything else is the default.
        if left_trigger_pressed and right_trigger_pressed:
            throttle_scale = self.teleop_turbo_scale
        elif left_trigger_pressed:
            throttle_scale = self.teleop_precision_scale
        else:
            throttle_scale = self.teleop_default_scale

        max_linear_speed = self.max_linear_speed * throttle_scale
        max_angular_speed = self.max_angular_speed * throttle_scale * self.teleop_rotation_scale

        # Deadband, then square the translation inputs to soften the low end of the stick.
        shaped_forward = self._apply_deadband(forward_speed, self.teleop_translation_deadband)
        shaped_strafe = self._apply_deadband(strafe_speed, self.teleop_translation_deadband)
        requested_vx = abs(shaped_forward) * shaped_forward * max_linear_speed
        requested_vy = abs(shaped_strafe) * shaped_strafe * max_linear_speed

        # Cubing rotation gives finer control near center than squaring, and keeps the sign.
        shaped_rotation = self._apply_deadband(rotation_speed, self.teleop_rotation_deadband)
        requested_omega = shaped_rotation * shaped_rotation * shaped_rotation * max_angular_speed

        # Translation is limited as a vector so the direction is kept; rotation is limited alone.
        limited_vx, limited_vy = self._slew_translation(requested_vx, requested_vy)
        limited_omega = self._slew_rotation(requested_omega)

        return (
            self.field_centric_request.with_velocity_x(limited_vx)
            .with_velocity_y(limited_vy)
            .with_rotational_rate(limited_omega)
        )

    @staticmethod
    def _apply_deadband(value: float, deadband: float) -> float:
        """
        Apply a deadband to a joystick value and rescale the remaining range to full travel.

        :param value: Raw joystick value in the range [-1, 1].
        :type value: float
        :param deadband: Deadband as a fraction of full travel.
        :type deadband: float
        :returns: Rescaled value that ramps from 0 at the deadband edge to 1 at full stick.
        :rtype: float
        """
        if abs(value) <= deadband:
            return 0.0

        return (value - copysign(deadband, value)) / (1.0 - deadband)

    def _slew_translation(self, requested_vx: float, requested_vy: float) -> tuple[float, float]:
        """
        Slew-rate limit the translation command as a vector so acceleration keeps its heading.

        :param requested_vx: Requested operator-perspective x velocity in meters per second.
        :type requested_vx: float
        :param requested_vy: Requested operator-perspective y velocity in meters per second.
        :type requested_vy: float
        :returns: The limited (vx, vy) command in meters per second.
        :rtype: tuple[float, float]
        """
        delta_vx = requested_vx - self._last_commanded_vx
        delta_vy = requested_vy - self._last_commanded_vy

        # Scale the whole delta down if it exceeds the per-loop limit, keeping its direction.
        delta_norm = hypot(delta_vx, delta_vy)
        if delta_norm > self.max_linear_rate_of_change:
            scale = self.max_linear_rate_of_change / delta_norm
            delta_vx *= scale
            delta_vy *= scale

        self._last_commanded_vx += delta_vx
        self._last_commanded_vy += delta_vy
        return self._last_commanded_vx, self._last_commanded_vy

    def _slew_rotation(self, requested_omega: float) -> float:
        """
        Slew-rate limit the rotation command toward the requested rate.

        :param requested_omega: Requested rotational rate in radians per second.
        :type requested_omega: float
        :returns: The limited rotational rate in radians per second.
        :rtype: float
        """
        delta_omega = requested_omega - self._last_commanded_omega
        if delta_omega > self.max_angular_rate_of_change:
            delta_omega = self.max_angular_rate_of_change
        elif delta_omega < -self.max_angular_rate_of_change:
            delta_omega = -self.max_angular_rate_of_change

        self._last_commanded_omega += delta_omega
        return self._last_commanded_omega

    def get_operator_drive_command(
        self,
        left_trigger: Callable[[], bool],
        right_trigger: Callable[[], bool],
        forward_speed: Callable[[], float],
        strafe_speed: Callable[[], float],
        rotation_speed: Callable[[], float],
    ):
        """
        Build the default teleop command that drives the robot from controller input.

        :param left_trigger: Function that returns whether the left trigger of the operator's
            controller is pressed or not.
        :type left_trigger: Callable[[], bool]
        :param right_trigger: Function that returns whether the right trigger of the operator's
            controller is pressed or not.
        :type right_trigger: Callable[[], bool]
        :param forward_speed: Function that returns the desired forward speed of the operator
            in terms of percent of max linear speed where forward is positive.
        :type forward_speed: Callable[[], float]
        :param strafe_speed: Function that returns the desired strafe speed of the operator
            in terms of percent of max linear speed where left is positive.
        :type strafe_speed: Callable[[], float]
        :param rotation_speed: Function that returns the desired rotation speed of the operator
            in terms of percent of max angular speed where counterclockwise is positive.
        :type rotation_speed: Callable[[], float]
        :returns: Command that drives the robot from live controller input.
        :rtype: commands2.Command
        """
        return self.run(
            lambda: self.set_control(
                self._get_operator_drive_request(
                    left_trigger(),
                    right_trigger(),
                    forward_speed(),
                    strafe_speed(),
                    rotation_speed(),
                )
            )
        )

    def _get_hub_center(self) -> Translation2d:
        """Return the hub center for the current alliance and field variant."""
        return get_hub_center(self.field_type, self.current_alliance)

    def _get_shooter_center_of_rotation(self) -> Translation2d:
        """Return the shooter offset used as the drivetrain center of rotation."""
        return self.shooter_offset.translation()

    def _get_hub_alignment_angle(self, current_pose: Pose2d) -> Rotation2d:
        """
        Find the heading that aims the shooter at the hub.

        The shooter sits off-center, so the aim is solved from the shooter's position rather than
        the robot's, then converted back into a robot heading.

        :param current_pose: Robot pose to solve the aim angle from.
        :type current_pose: wpimath.geometry.Pose2d
        """
        shooter_pose = current_pose.transformBy(self.shooter_offset)
        hub_center = self._get_hub_center()

        aim_angle = Rotation2d(
            hub_center.x - shooter_pose.x,
            hub_center.y - shooter_pose.y,
        )

        return aim_angle - self.shooter_offset.rotation()

    def _apply_alignment_target(self, target_angle: Rotation2d):
        """
        Rotate the robot toward a target heading, pivoting about the shooter.

        Pivoting about the shooter instead of the robot's center keeps the shooter roughly in
        place while the robot turns, so the shot distance barely changes during alignment.

        :param target_angle: Field-relative heading to rotate the robot toward.
        :type target_angle: wpimath.geometry.Rotation2d
        """
        request = self.rotate_robot_request.with_target_direction(
            target_angle
        ).with_center_of_rotation(self._get_shooter_center_of_rotation())

        self.set_control(request)

    def get_hub_alignment_error(self) -> Rotation2d:
        """
        Return the signed heading error between the robot and the hub aim angle.

        :returns: How far the robot still has to turn to face the hub.
        :rtype: wpimath.geometry.Rotation2d
        """
        current_pose = self.get_state().pose
        return self._get_hub_alignment_angle(current_pose) - current_pose.rotation()

    def is_hub_alignment_within_tolerance(self, tolerance_deg: float = 2.5) -> bool:
        """
        Check whether hub alignment error is within the requested tolerance.

        :param tolerance_deg: Maximum allowed absolute hub alignment error in degrees.
        :type tolerance_deg: float
        :returns: True when the current hub alignment error is within tolerance.
        :rtype: bool
        """
        return abs(self.get_hub_alignment_error().degrees()) <= tolerance_deg

    def create_hold_hub_alignment_command(self):
        """
        Build a command that keeps the robot turned toward the hub.

        The heading controller is reset first so it does not act on error left over from an
        earlier alignment.

        :returns: Command that holds hub alignment until it is interrupted.
        :rtype: commands2.Command
        """
        return self.runOnce(lambda: self.rotate_robot_pid_controller.reset()).andThen(
            self.run(
                lambda: self._apply_alignment_target(
                    self._get_hub_alignment_angle(self.get_state().pose)
                )
            )
        )

    def create_stop_command(self):
        """
        Build a one-shot command that zeroes all chassis velocity requests.

        :returns: Command that stops the drivetrain.
        :rtype: commands2.Command
        """
        return self.runOnce(
            lambda: self.set_control(
                self.field_centric_request.with_velocity_x(0.0)
                .with_velocity_y(0.0)
                .with_rotational_rate(0.0)
            )
        )

    def create_module_alignment_diagnostic_command(self):
        """
        Point all modules forward and publish each module's azimuth error for calibration.

        Run this in test mode to check that the modules really point straight when commanded to
        0 degrees. A nonzero error that differs per module usually means the CANcoder offsets
        need recalibrating, which is the common cause of the robot yawing while driving straight.

        :returns: Command that holds the modules forward and reports their errors.
        :rtype: commands2.Command
        """
        point_forward_request = swerve.requests.PointWheelsAt().with_module_direction(Rotation2d())

        def publish_module_errors():
            """Command the modules forward and report each module's azimuth error in degrees."""
            self.set_control(point_forward_request)
            for module_index in range(len(self.modules)):
                current_angle = self.get_module(module_index).get_current_state().angle
                SmartDashboard.putNumber(
                    f"Drivetrain/Module {module_index} Azimuth Error (deg)",
                    current_angle.degrees(),
                )

        return self.run(publish_module_errors)

    def create_effective_wheel_radius_characterization_command(self):
        """
        Spin the robot in place to measure the wheel radius the drivetrain actually has.

        Comparing how far the wheels report travelling against how far the robot really turned
        gives the true radius, which drifts from the nominal value as tread wears down. Feeding
        the measured value back into the constants keeps odometry accurate.

        :returns: Command that runs the spin test and publishes the measured radius.
        :rtype: commands2.Command
        """
        ramp_duration_sec = 2.0
        hold_duration_sec = 1.0
        characterization_duration_sec = (2.0 * ramp_duration_sec) + hold_duration_sec
        initial_yaw_deg = 0.0
        initial_distances_m = [0.0] * len(self.modules)
        timer = Timer()

        def initialize():
            """Capture the starting yaw and module distances before the spin test."""
            nonlocal initial_yaw_deg, initial_distances_m
            initial_yaw_deg = self.pigeon2.get_yaw().value_as_double
            initial_distances_m = [
                self.get_module(module_index).get_position(True).distance
                for module_index in range(len(self.modules))
            ]
            timer.restart()

        def execute():
            """Ramp up, hold, then ramp down the spin so the wheels do not slip."""
            elapsed_sec = timer.get()
            if elapsed_sec < ramp_duration_sec:
                requested_omega_rad_per_sec = (self.max_angular_speed * 0.5) * (
                    elapsed_sec / ramp_duration_sec
                )
            elif elapsed_sec < (ramp_duration_sec + hold_duration_sec):
                requested_omega_rad_per_sec = self.max_angular_speed * 0.5
            else:
                requested_omega_rad_per_sec = (self.max_angular_speed * 0.5) * max(
                    0.0,
                    (characterization_duration_sec - elapsed_sec) / ramp_duration_sec,
                )

            self.set_control(
                self.field_centric_request.with_velocity_x(0.0)
                .with_velocity_y(0.0)
                .with_rotational_rate(-requested_omega_rad_per_sec)
            )

        def end(interrupted: bool):
            """
            Stop the robot and print the computed effective wheel radius.

            :param interrupted: Whether the characterization command ended due to interruption.
            :type interrupted: bool
            """
            timer.stop()
            self.set_control(
                self.field_centric_request.with_velocity_x(0.0)
                .with_velocity_y(0.0)
                .with_rotational_rate(0.0)
            )

            final_yaw_deg = self.pigeon2.get_yaw().value_as_double
            yaw_delta_rad = abs(final_yaw_deg - initial_yaw_deg) * pi / 180.0
            wheel_distance_delta_m = [
                abs(
                    self.get_module(module_index).get_position(True).distance
                    - initial_distances_m[module_index]
                )
                for module_index in range(len(self.modules))
            ]
            average_wheel_delta_m = sum(wheel_distance_delta_m) / len(wheel_distance_delta_m)

            # Both deltas divide into the result, so a robot that never moved would produce a
            # meaningless number rather than a measurement.
            if average_wheel_delta_m <= 1e-9 or yaw_delta_rad <= 1e-9:
                reportWarning(
                    "Effective wheel radius characterization did not collect enough movement data.",
                    False,
                )
                return

            effective_wheel_radius_m = (
                self.wheel_radius * self._drive_base_radius * yaw_delta_rad / average_wheel_delta_m
            )
            SmartDashboard.putNumber(
                "Drivetrain/Effective Wheel Radius (in)", effective_wheel_radius_m / 0.0254
            )

        return FunctionalCommand(
            initialize,
            execute,
            end,
            lambda: timer.get() >= characterization_duration_sec,
            self,
        )
