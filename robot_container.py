import commands2
from commands2.sysid import SysIdRoutine
from pathplannerlib.auto import AutoBuilder, NamedCommands
from wpilib import DriverStation, SendableChooser, SmartDashboard, reportWarning

from constants.hopper_constants import HopperConstants
from constants.intake_constants import IntakeConstants
from constants.led_constants import LEDConstants
from constants.shooter_constants import ShooterConstants
from constants.shot_calculator_constants import get_hub_center
from constants.swerve_drivetrain_constants import SwerveDrivetrainConstants
from constants.vision_constants import VisionConstants


class RobotContainer:
    """
    Holds the robot's subsystems and wires them to the controller and the dashboard.

    Everything the robot owns is built here, so this is the one place to look for what a button
    does or which subsystems a command touches.
    """

    # Trigger travel past which a controller trigger counts as pressed
    _TRIGGER_DEADBAND = 0.10

    # Remaining teleop seconds at which the endgame LED warning starts
    _ENDGAME_WARNING_SEC = 5

    def __init__(self):
        """Build the subsystems, dashboard choosers, and controller bindings."""
        # Create controller
        self.controller = commands2.button.CommandXboxController(0)

        # Create LED subsystem
        self.led = LEDConstants.create_led()

        # Create hopper subsystem
        self.hopper = HopperConstants.create_hopper()

        # Create intake subsystem. The hopper and intake are built before the drivetrain on
        # purpose: their configs retry through the several seconds after code start when Phoenix
        # license checks can reject configs on the CANivore, so by the time CTRE's swerve
        # constructor applies its own configs (which retry far less) that window has passed.
        self.intake = IntakeConstants.create_intake()

        # Create drivetrain subsystem
        self.drivetrain = SwerveDrivetrainConstants.create_drivetrain()

        launcher_offset = self.drivetrain.shooter_offset.translation()

        # Create shooter subsystem
        self.shooter = ShooterConstants.create_shooter(
            self.drivetrain.get_state,
            lambda: get_hub_center(
                self.drivetrain.field_type,
                self.drivetrain.current_alliance,
            ),
            launcher_offset.x,
            launcher_offset.y,
        )

        # Create vision subsystem
        self.camera = VisionConstants.create_vision(
            self.drivetrain.add_vision_measurement,
            self.drivetrain.get_state,
            self.drivetrain.get_robot_tilt,
            self.drivetrain.set_camera_pose,
        )

        self.register_named_commands()

        # The chooser can only be built once AutoBuilder is configured, which the drivetrain
        # skips when the PathPlanner robot config fails to load. An empty chooser keeps the
        # dashboard usable instead of crashing startup.
        if AutoBuilder.isConfigured():
            self.auto_chooser = AutoBuilder.buildAutoChooser()
        else:
            self.auto_chooser = SendableChooser()
            reportWarning("PathPlanner not configured; auto chooser is empty", False)
        SmartDashboard.putData("Auto Chooser", self.auto_chooser)

        # Every mechanism the test-mode SysId buttons can characterize, gathered into one
        # chooser. Each option names both the subsystem and the routine, so there is no way to
        # run a characterization on a mechanism other than the one shown.
        sys_id_routines = [
            (f"{subsystem_name} - {routine_name}", routine)
            for subsystem_name, subsystem in (
                ("Drivetrain", self.drivetrain),
                ("Shooter", self.shooter),
                ("Hopper", self.hopper),
            )
            for routine_name, routine in subsystem.sys_id_routines
        ]

        default_option_name, default_routine = sys_id_routines[0]
        self.sys_id_routine_chooser = SendableChooser()
        self.sys_id_routine_chooser.setDefaultOption(default_option_name, default_routine)
        for option_name, routine in sys_id_routines[1:]:
            self.sys_id_routine_chooser.addOption(option_name, routine)

        SmartDashboard.putData("SysId Routine", self.sys_id_routine_chooser)

        self.create_button_bindings()

    def get_selected_auto_command(self):
        """Return the autonomous command selected on the dashboard chooser."""
        return self.auto_chooser.getSelected()

    def register_named_commands(self):
        """
        Register the commands PathPlanner paths can call by name.

        These names are what the path editor lists as event markers, so they must match the
        names used in the deployed paths.
        """
        NamedCommands.registerCommand(
            "Run Intake",
            self.intake.runOnce(
                lambda: self.intake.set_intake_speed(IntakeConstants._intake_volts)
            ),
        )

        NamedCommands.registerCommand(
            "Turn Off Intake", self.intake.runOnce(lambda: self.intake.set_intake_speed(0))
        )

        NamedCommands.registerCommand(
            "Auto Run Shooter",
            commands2.ParallelCommandGroup(
                commands2.SequentialCommandGroup(
                    commands2.WaitCommand(ShooterConstants._auto_arm_down_delay_sec),
                    self.intake.runOnce(lambda: self.intake.arm_down_intermediate()),
                ),
                # The same aligned, distance-based shot as teleop B, ending when the hopper runs
                # empty since there is no driver to stop it.
                self.shooter.create_shot_command(
                    self.hopper,
                    self.drivetrain,
                    distance_based=True,
                    end_when=lambda: self.shooter.detect_empty(),
                ),
            ),
        )

    def create_commands_auto(self):
        """Put every subsystem into a known safe state before autonomous begins."""
        self.intake.arm_down()
        self.intake.set_intake_speed(0)
        self.hopper.set_hopper_speeds(0, 0)
        self.shooter.set_flywheel_velocities(0, 0)
        self.led.auto_in_progress()

    def create_commands_teleop(self):
        """Reset driver-facing state and zero every subsystem output for teleop."""
        # The alliance may only be known once connected to the FMS, so the driving perspective
        # is set here rather than at startup.
        self.drivetrain.set_forward_perspective()
        self.drivetrain.reset_teleop_drive_state()

        self.intake.arm_down()
        self.intake.set_intake_speed(0)
        self.hopper.set_hopper_speeds(0, 0)
        self.shooter.set_flywheel_velocities(0, 0)
        self.led.pride()

    def create_button_bindings(self):
        """
        Bind controller buttons to robot actions and default commands.

        All bindings are created once here and gated by a teleop or test trigger, so a binding
        only fires in its own mode and re-entering a mode does not stack duplicates. The color
        buttons shoot in teleop and run SysId characterization in test.
        """
        # A mode-specific binding is only live while its mode is enabled.
        teleop = commands2.button.Trigger(lambda: DriverStation.isTeleopEnabled())
        test = commands2.button.Trigger(lambda: DriverStation.isTestEnabled())

        # Driving is the drivetrain's default command, so it resumes whenever nothing else
        # (auto-align, a path, a characterization) is using the drivetrain.
        self.drivetrain.setDefaultCommand(
            self.drivetrain.get_operator_drive_command(
                lambda: self.controller.getLeftTriggerAxis() > self._TRIGGER_DEADBAND,
                lambda: self.controller.getRightTriggerAxis() > self._TRIGGER_DEADBAND,
                lambda: -self.controller.getLeftY(),
                lambda: -self.controller.getLeftX(),
                lambda: -self.controller.getRightX(),
            ).beforeStarting(
                self.drivetrain.runOnce(lambda: self.drivetrain.reset_teleop_drive_state())
            )
        )

        # --- Teleop bindings ---

        # Both bumpers together re-zero field-centric driving to wherever the robot is pointing.
        def reset_field_centric_heading():
            self.drivetrain.seed_field_centric()
            self.drivetrain.set_forward_perspective()
            self.drivetrain.reset_teleop_drive_state()

        (self.controller.leftBumper() & self.controller.rightBumper() & teleop).onTrue(
            self.drivetrain.runOnce(reset_field_centric_heading)
        )

        # A toggles the intake. The wait keeps a quick double-press from cancelling the intake
        # before it has had a chance to pull anything in.
        (self.controller.a() & teleop).onTrue(
            commands2.SequentialCommandGroup(
                self.intake.runOnce(
                    lambda: self.intake.set_intake_speed(IntakeConstants._intake_volts)
                ),
                commands2.WaitCommand(IntakeConstants._intake_min_run_sec),
                commands2.WaitUntilCommand(lambda: self.controller.getHID().getAButton()),
                self.intake.runOnce(lambda: self.intake.set_intake_speed(0)),
            )
        )

        # D-pad right reverses the intake, hopper, and flywheels together to clear a jam.
        (self.controller.povRight() & teleop).onTrue(
            commands2.ParallelCommandGroup(
                self.intake.runOnce(
                    lambda: self.intake.set_intake_speed(IntakeConstants._eject_volts)
                ),
                self.hopper.run_hopper(
                    HopperConstants._eject_mecanum_velocity,
                    HopperConstants._eject_agitator_volts,
                ),
                self.shooter.runOnce(
                    lambda: self.shooter.set_flywheel_velocities(
                        ShooterConstants._eject_flywheel_velocity,
                        ShooterConstants._eject_flywheel_velocity,
                    )
                ),
            )
        )

        # D-pad left re-seeds odometry from the hub after the pose has drifted.
        (self.controller.povLeft() & teleop).onTrue(
            self.drivetrain.runOnce(lambda: self.drivetrain.reset_pose_hub())
        )

        # Each press steps the arm one preset (intake <-> shooting <-> stowed) in that
        # direction, and both moves watch for a stall so a jammed arm stops instead of
        # straining. The target resolves at press time, so a press during a move steps from
        # the position already commanded.
        (self.controller.povDown() & teleop).onTrue(
            self._create_arm_move_command(lambda: self.intake.arm_down())
        )

        (self.controller.povUp() & teleop).onTrue(
            self._create_arm_move_command(lambda: self.intake.step_arm_up())
        )

        # X fires a manual shot: dashboard flywheel speed, no auto-align, ends on Y.
        (self.controller.x() & teleop).onTrue(
            commands2.SequentialCommandGroup(
                self.led.runOnce(lambda: self.led.shooting_manual()),
                self.intake.runOnce(
                    lambda: self.intake.set_intake_speed(IntakeConstants._intake_volts)
                ),
                self.shooter.create_shot_command(
                    self.hopper,
                    distance_based=False,
                    end_when=lambda: self.controller.getHID().getYButton(),
                ),
                self._create_stop_all_command(),
            )
        )

        # B fires a calculated shot: distance-based flywheel speed while auto-aligning to the
        # hub, ending on Y or when the hopper runs empty.
        (self.controller.b() & teleop).onTrue(
            commands2.SequentialCommandGroup(
                self.led.runOnce(lambda: self.led.shooting_calculated()),
                self.intake.runOnce(
                    lambda: self.intake.set_intake_speed(IntakeConstants._intake_volts)
                ),
                self.shooter.create_shot_command(
                    self.hopper,
                    self.drivetrain,
                    distance_based=True,
                    end_when=lambda: (
                        self.controller.getHID().getYButton() or self.shooter.detect_empty()
                    ),
                ),
                self._create_stop_all_command(),
            )
        )

        # Y is the panic stop for everything shooting-related.
        (self.controller.y() & teleop).onTrue(self._create_stop_all_command())

        # Flash the LEDs for the last five seconds of teleop, then go back to the default state.
        # getMatchTime() returns -1 without a real timing source, so this also requires an FMS or
        # a practice match to avoid firing during a bench test.
        five_seconds_left = commands2.button.Trigger(
            lambda: (
                DriverStation.isTeleopEnabled()
                and 0 < DriverStation.getMatchTime() <= self._ENDGAME_WARNING_SEC
                and (
                    DriverStation.isFMSAttached()
                    or DriverStation.getMatchType() == DriverStation.MatchType.kPractice
                )
            )
        )
        five_seconds_left.onTrue(self.led.runOnce(lambda: self.led.five_seconds_left())).onFalse(
            self.led.runOnce(lambda: self.led.default())
        )

        # --- Test bindings (SysId characterization + drivetrain diagnostics) ---

        # Color buttons run SysId on whichever routine the dashboard chooser is set to.
        (self.controller.y() & test).whileTrue(
            self._create_sys_id_command(
                lambda routine: routine.quasistatic(SysIdRoutine.Direction.kForward)
            )
        )
        (self.controller.a() & test).whileTrue(
            self._create_sys_id_command(
                lambda routine: routine.quasistatic(SysIdRoutine.Direction.kReverse)
            )
        )
        (self.controller.b() & test).whileTrue(
            self._create_sys_id_command(
                lambda routine: routine.dynamic(SysIdRoutine.Direction.kForward)
            )
        )
        (self.controller.x() & test).whileTrue(
            self._create_sys_id_command(
                lambda routine: routine.dynamic(SysIdRoutine.Direction.kReverse)
            )
        )

        (self.controller.povUp() & test).onTrue(
            self.drivetrain.create_effective_wheel_radius_characterization_command()
        )

        # Hold the modules forward and publish their azimuth errors for offset calibration.
        (self.controller.povDown() & test).whileTrue(
            self.drivetrain.create_module_alignment_diagnostic_command()
        )

    def _create_arm_move_command(self, arm_action):
        """
        Move the intake arm to a position while watching for a stall.

        The stall watch runs until the arm reaches its target or jams, so a blocked arm stops
        driving instead of straining against whatever is in its way.

        :param arm_action: Callable that commands the arm toward its target position.
        :type arm_action: Callable[[], None]
        :returns: Command that runs the arm move with stall detection.
        :rtype: commands2.Command
        """

        # run needs a None-returning action, so the stall check's result is discarded here.
        def watch_for_stall() -> None:
            self.intake.get_stall_detection()

        return commands2.SequentialCommandGroup(
            self.intake.runOnce(arm_action),
            # The timeout is a backstop for a move that neither finishes nor trips the stall
            # watch, so the arm cannot keep driving forever.
            self.intake.run(watch_for_stall)
            .until(lambda: self.intake.detect_arm_movement_completion())
            .withTimeout(IntakeConstants._arm_move_timeout_sec),
        )

    def _create_stop_all_command(self):
        """
        Stop the intake, hopper, and shooter, and return the LEDs to their default state.

        This is the idle state a shot ends in and the state the Y button forces at any time.

        :returns: Command that puts every shooting-related subsystem back to idle.
        :rtype: commands2.Command
        """
        return commands2.ParallelCommandGroup(
            self.intake.runOnce(lambda: self.intake.set_intake_speed(0)),
            self.hopper.create_stop_command(),
            self.shooter.create_stop_command(),
            self.led.runOnce(lambda: self.led.pride()),
        )

    def _create_sys_id_command(self, run_routine):
        """
        Build a deferred SysId command targeting the dashboard-selected routine.

        The routine is resolved when the button is pressed, so the chooser can be changed between
        runs. Every characterizable subsystem is required up front so whichever routine is chosen
        already holds the subsystem it needs.

        :param run_routine: Callable mapping the selected routine to the SysId command to run
            (quasistatic/dynamic in a given direction).
        :type run_routine: Callable[[commands2.sysid.SysIdRoutine], commands2.Command]
        :returns: Deferred command that runs the selected SysId routine.
        :rtype: commands2.Command
        """

        def build():
            return run_routine(self.sys_id_routine_chooser.getSelected())

        return commands2.DeferredCommand(build, self.drivetrain, self.shooter, self.hopper)

    def create_commands_test(self):
        """
        Zero every subsystem output for test mode.

        The test-mode button bindings themselves live in create_button_bindings, scoped to test
        mode, so nothing needs to be re-bound here.
        """
        self.intake.set_intake_speed(0)
        self.hopper.set_hopper_speeds(0, 0)
        self.shooter.set_flywheel_velocities(0, 0)
        self.led.default()
