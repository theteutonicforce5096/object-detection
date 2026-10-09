from typing import Final

from commands2 import Subsystem
from phoenix6 import CANBus
from phoenix6.configs import CANcoderConfiguration, TalonFXConfiguration, TalonFXSConfiguration
from phoenix6.controls import PositionVoltage, VoltageOut
from phoenix6.hardware import CANcoder, TalonFX, TalonFXS
from wpilib import RobotBase, SmartDashboard, Timer

from subsystems.device_config import check_signal_status, configure_device


class Intake(Subsystem):
    """
    Picks balls up off the floor and holds the pivoting arm that carries the intake wheel.

    The arm runs closed-loop to a handful of preset positions using a fused CANcoder for absolute
    feedback. Because the arm can jam against the frame or an obstruction, a stall watch cuts
    power when a commanded arm stops making progress toward its target.
    """

    # Rotations from a target position within which the arm counts as having arrived
    _ARM_POSITION_TOLERANCE_ROT: Final = 0.009

    # Minimum position error at which a stall may be declared. Inside this band the closed-loop
    # output tapers off and the arm naturally slows below the stall velocity threshold, which
    # must not read as a jam. A jam inside this final sliver is ended by the arm-move command
    # timeout instead.
    _STALL_POSITION_ERROR_MIN_ROT: Final = 0.009

    def __init__(
        self,
        canbus: CANBus,
        intake_wheel_id: int,
        intake_arm_id: int,
        intake_arm_encoder_id: int,
        intake_wheel_configs: TalonFXSConfiguration,
        intake_arm_configs: TalonFXConfiguration,
        intake_arm_encoder_configs: CANcoderConfiguration,
        num_config_attempts: int,
        intake_position: float,
        stowed_position: float,
        shooting_position: float,
        stall_current_threhsold: float,
        stall_velocity_threshold: float,
        stall_time_threshold: float,
    ):
        """
        Initialize the intake using the specified constants.

        :param canbus: CANBus instance that electronics are on
        :type canbus: phoenix6.CANBus
        :param intake_wheel_id: CAN ID of the intake wheel
        :type intake_wheel_id: int
        :param intake_arm_id: CAN ID of the intake arm
        :type intake_arm_id: int
        :param intake_arm_encoder_id: CAN ID of the intake arm encoder
        :type intake_arm_encoder_id: int
        :param intake_wheel_configs: Configs for the intake wheel
        :type intake_wheel_configs: phoenix6.configs.TalonFXSConfiguration
        :param intake_arm_configs: Configs for the intake arm
        :type intake_arm_configs: phoenix6.configs.TalonFXConfiguration
        :param intake_arm_encoder_configs: Configs for the intake arm encoder
        :type intake_arm_encoder_configs: phoenix6.configs.CANcoderConfiguration
        :param num_config_attempts: Number of times to attempt to configure each device
        :type num_config_attempts: int
        :param intake_position: Encoder position where arm is down
        :type intake_position: float
        :param stowed_position: Encoder position where arm is up
        :type stowed_position: float
        :param shooting_position: Encoder position where the arm is at an intermediate position
        :type shooting_position: float
        :param stall_velocity_threshold: Arm velocity below which the arm counts as stalled
        :type stall_velocity_threshold: float
        :param stall_time_threshold: Time the stall conditions must hold before a stall is
            declared
        :type stall_time_threshold: float
        """
        Subsystem.__init__(self)

        # Create motors
        self.intake_wheel = TalonFXS(intake_wheel_id, canbus)
        self.intake_arm = TalonFX(intake_arm_id, canbus)
        self.intake_arm_encoder = CANcoder(intake_arm_encoder_id, canbus)

        # Apply motor configs
        configure_device(self.intake_wheel, intake_wheel_configs, num_config_attempts)
        configure_device(self.intake_arm, intake_arm_configs, num_config_attempts)
        configure_device(self.intake_arm_encoder, intake_arm_encoder_configs, num_config_attempts)

        if not RobotBase.isSimulation():
            # Trim every signal, then raise only the arm signals the stall and move-complete
            # checks poll each loop. The encoder is fused into the arm on-device, so its own
            # frame is only used for logging.
            check_signal_status(
                self.intake_wheel.optimize_bus_utilization(),
                "Intake wheel bus optimization",
            )
            check_signal_status(
                self.intake_arm.optimize_bus_utilization(),
                "Intake arm bus optimization",
            )
            check_signal_status(
                self.intake_arm_encoder.optimize_bus_utilization(),
                "Intake arm encoder bus optimization",
            )
            check_signal_status(
                self.intake_arm.get_position().set_update_frequency(100.0),
                "Intake arm position update rate",
            )
            check_signal_status(
                self.intake_arm.get_velocity().set_update_frequency(100.0),
                "Intake arm velocity update rate",
            )
            # Stator current is only published for dashboard tuning, so it does not need the
            # high rate the control-loop signals run at.
            check_signal_status(
                self.intake_arm.get_stator_current().set_update_frequency(20.0),
                "Intake arm stator current update rate",
            )
            check_signal_status(
                self.intake_arm_encoder.get_position().set_update_frequency(100.0),
                "Intake arm encoder position update rate",
            )

        # Create control requests. The wheel and the arm each get their own voltage request so
        # one mechanism's command can never carry the other's stale output.
        self.wheel_voltage_request = VoltageOut(output=0)
        self.arm_voltage_request = VoltageOut(output=0)
        self.position_voltage_request = PositionVoltage(position=0)

        # Arm positions
        self.intake_position = intake_position
        self.stowed_position = stowed_position
        self.shooting_position = shooting_position

        # None until the arm has been commanded somewhere, which the stall and move-complete
        # checks both treat as "nothing to watch"
        self.commanded_position: float | None = None
        self.is_stalled = False

        # Stall detection tunables
        self.stall_current_threshold = stall_current_threhsold
        self.stall_velocity_threshold = stall_velocity_threshold
        self.stall_time_threshold = stall_time_threshold
        self.stall_timer = Timer()

        # Arm signals refreshed each loop for stall detection and telemetry
        self.arm_stator_current = 0.0
        self.arm_velocity = 0.0
        self.arm_position = 0.0

    def periodic(self):
        """Refresh the arm signals used for stall detection and report them to the dashboard."""
        self.arm_stator_current = self.intake_arm.get_stator_current().value_as_double
        self.arm_velocity = abs(self.intake_arm.get_velocity().value_as_double)
        self.arm_position = self.intake_arm.get_position().value_as_double

        # The stall watch runs every loop, not just while an arm-move command is scheduled, so
        # a jam still cuts power even when nothing else is polling it. It runs before is_stalled
        # is published so the dashboard shows this loop's verdict.
        self.get_stall_detection()

        # Velocity and position are the signals the stall logic runs on, and current shows how
        # hard the arm is working; watching the real values is what makes the stall thresholds
        # and current limit tunable from measurements instead of guesses.
        SmartDashboard.putBoolean("Intake/Arm Stalled", self.is_stalled)
        SmartDashboard.putNumber("Intake/Arm Current (A)", self.arm_stator_current)
        SmartDashboard.putNumber("Intake/Arm Velocity (rps)", self.arm_velocity)
        SmartDashboard.putNumber("Intake/Arm Position (rot)", self.arm_position)

        # NetworkTables has no way to say "no value", so an idle arm reports -1 instead of a
        # position it was never commanded to.
        SmartDashboard.putNumber(
            "Intake/Commanded Position (rot)",
            self.commanded_position if self.commanded_position is not None else -1.0,
        )

    def set_intake_speed(self, intake_wheel_volts):
        """
        Apply the requested intake-wheel voltage.

        :param intake_wheel_volts: Voltage to apply to the intake wheel motor.
        :type intake_wheel_volts: float
        """
        self.intake_wheel.set_control(self.wheel_voltage_request.with_output(intake_wheel_volts))

    def set_setpoint(self, position):
        """
        Command the intake arm to the requested closed-loop position.

        :param position: Desired intake arm position in mechanism rotations.
        :type position: float
        """
        self.commanded_position = position

        # A new move starts clean, so clear any stall left over from the previous move before
        # commanding the arm again.
        self.is_stalled = False
        self.stall_timer.stop()
        self.stall_timer.reset()

        self.intake_arm.set_control(self.position_voltage_request.with_position(position))

    def detect_arm_movement_completion(self):
        """
        Report whether the arm has finished its move.

        :returns: True once the arm reaches the commanded position or a stall is recorded.
        :rtype: bool
        """
        if self.commanded_position is None:
            return False
        return (
            self.intake_arm.get_position().is_near(
                self.commanded_position, self._ARM_POSITION_TOLERANCE_ROT
            )
            or self.is_stalled
        )

    def set_arm_voltage(self, voltage):
        """
        Drive the intake arm with an open-loop voltage.

        :param voltage: Voltage to apply to the arm motor.
        :type voltage: float
        """
        self.intake_arm.set_control(self.arm_voltage_request.with_output(voltage))

    def arm_down(self):
        """Move the intake arm down to the floor-pickup position."""
        self.set_setpoint(self.intake_position)

    def arm_up(self):
        """Move the intake arm up to the stowed position."""
        self.set_setpoint(self.stowed_position)

    def arm_down_intermediate(self):
        """Move the intake arm to the intermediate position used while shooting."""
        self.set_setpoint(self.shooting_position)

    def step_arm_up(self):
        """Move the intake arm up one preset position toward stowed."""
        self.set_setpoint(self._next_arm_position(moving_up=True))

    def step_arm_down(self):
        """Move the intake arm down one preset position toward intake."""
        self.set_setpoint(self._next_arm_position(moving_up=False))

    def _next_arm_position(self, moving_up: bool) -> float:
        """
        Return the preset position one step above or below the arm's reference position.

        The reference is the last commanded position, or the measured position when nothing has
        been commanded yet (such as after a code restart), so an arm resting between presets
        still steps to the next preset in the requested direction. The presets bound the travel:
        stepping up from stowed or down from intake returns that same bound.

        :param moving_up: True to step toward the stowed position, False toward intake.
        :type moving_up: bool
        :returns: Preset position one step in the requested direction.
        :rtype: float
        """
        reference = (
            self.commanded_position if self.commanded_position is not None else self.arm_position
        )
        presets = (self.intake_position, self.shooting_position, self.stowed_position)

        # The tolerance keeps the preset the arm is already at from being chosen again.
        if moving_up:
            candidates = [p for p in presets if p > reference + self._ARM_POSITION_TOLERANCE_ROT]
            return min(candidates) if candidates else self.stowed_position
        candidates = [p for p in presets if p < reference - self._ARM_POSITION_TOLERANCE_ROT]
        return max(candidates) if candidates else self.intake_position

    def get_stall_detection(self):
        """
        Watch the moving arm and cut power if it stops making progress.

        A stall is any lack of movement while the arm is commanded well short of its target:
        the arm barely moves for long enough to rule out normal acceleration. Detection is
        purely movement-based, so it catches a jam anywhere in the travel regardless of how
        much current the obstruction happens to draw.

        :returns: True once a stall has been detected, otherwise False.
        :rtype: bool
        """
        if self.commanded_position is None:
            return False

        # A stall latches until the next setpoint clears it, so a jammed arm stays stopped
        # instead of re-driving the moment it springs back.
        if self.is_stalled:
            return True

        # Stall: still commanding a move, but hardly moving. The wider error band keeps the
        # closed-loop taper near the target from reading as a stall (see the class constant).
        is_commanding_motion = (
            abs(self.commanded_position - self.arm_position) > self._STALL_POSITION_ERROR_MIN_ROT
        )
        stall_condition_met = (
            is_commanding_motion and self.arm_velocity < self.stall_velocity_threshold and self.arm_stator_current > self.stall_current_threshold
        )

        # The timer only runs while the conditions hold, so a brief hesitation during normal
        # acceleration resets it instead of counting toward a stall.
        if stall_condition_met:
            self.stall_timer.start()
        else:
            self.stall_timer.stop()
            self.stall_timer.reset()

        self.is_stalled = (
            self.stall_timer.hasElapsed(self.stall_time_threshold) and stall_condition_met
        )

        if self.is_stalled:
            self.set_arm_voltage(0)

        return self.is_stalled