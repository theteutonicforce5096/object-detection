from phoenix6 import CANBus, signals
from phoenix6.configs import TalonFXConfiguration, TalonFXSConfiguration

from subsystems.shooter import Shooter


class ShooterConstants:
    """Hardware IDs, motor configs, and shot tuning values for the shooter."""

    # CANBus instance
    _canbus = CANBus("Drivetrain")

    # CAN IDs
    _flywheel_motor_id = 30
    _flywheel_intake_motor_id = 31

    # Number of times to attempt to configure each device. Sized so retries span the several
    # seconds after code start when Phoenix license checks can reject configs on a pro bus.
    _num_config_attempts = 20

    # Flywheel Motor Configs (NEO VORTEX - TalonFXS)
    _flywheel_motor_configs = TalonFXSConfiguration()
    _flywheel_motor_configs.commutation.with_motor_arrangement(
        signals.MotorArrangementValue.VORTEX_JST
    )
    _flywheel_motor_configs.commutation.with_advanced_hall_support(
        signals.AdvancedHallSupportValue.ENABLED
    )
    _flywheel_motor_configs.motor_output.with_neutral_mode(signals.NeutralModeValue.BRAKE)
    _flywheel_motor_configs.motor_output.with_inverted(signals.InvertedValue.CLOCKWISE_POSITIVE)
    _flywheel_motor_configs.current_limits.with_stator_current_limit(80)
    _flywheel_motor_configs.current_limits.with_stator_current_limit_enable(True)
    # Supply limit shaped for shot recovery: each ball costs a burst of tens of milliseconds,
    # well inside the higher budget, and the draw dips between balls reset the lower-limit
    # timer, so rapid-fire recovery keeps full torque while sustained draw is clamped.
    _flywheel_motor_configs.current_limits.with_supply_current_limit(60)
    _flywheel_motor_configs.current_limits.with_supply_current_limit_enable(True)
    _flywheel_motor_configs.current_limits.with_supply_current_lower_limit(30)
    _flywheel_motor_configs.current_limits.with_supply_current_lower_time(1.0)
    _flywheel_motor_configs.slot0.with_k_s(0.11618)
    _flywheel_motor_configs.slot0.with_k_v(0.10922)
    _flywheel_motor_configs.slot0.with_k_p(0.1275)
    _flywheel_motor_configs.slot0.with_k_i(0)
    _flywheel_motor_configs.slot0.with_k_d(0)

    # Flywheel Intake Motor Configs (KrakenX60 - TalonFX)
    _flywheel_intake_motor_configs = TalonFXConfiguration()
    _flywheel_intake_motor_configs.motor_output.with_neutral_mode(signals.NeutralModeValue.BRAKE)
    _flywheel_intake_motor_configs.motor_output.with_inverted(
        signals.InvertedValue.CLOCKWISE_POSITIVE
    )
    _flywheel_intake_motor_configs.current_limits.with_stator_current_limit(80)
    _flywheel_intake_motor_configs.current_limits.with_stator_current_limit_enable(True)
    # Same burst-then-clamp shape as the flywheel so feeding keeps its torque headroom while
    # a stalled feed cannot hold high battery draw
    _flywheel_intake_motor_configs.current_limits.with_supply_current_limit(40)
    _flywheel_intake_motor_configs.current_limits.with_supply_current_limit_enable(True)
    _flywheel_intake_motor_configs.current_limits.with_supply_current_lower_limit(20)
    _flywheel_intake_motor_configs.current_limits.with_supply_current_lower_time(1.0)
    _flywheel_intake_motor_configs.slot0.with_k_s(0.31895)
    _flywheel_intake_motor_configs.slot0.with_k_v(0.11374)
    _flywheel_intake_motor_configs.slot0.with_k_p(0.75)
    _flywheel_intake_motor_configs.slot0.with_k_i(0)
    _flywheel_intake_motor_configs.slot0.with_k_d(0)

    # Flywheel speed offset in rotations per second added to every shot target. Raise it if
    # shots consistently land short, lower it if they consistently overshoot.
    _flywheel_target_offset_rps = 1.0

    # Reverse flywheel speed used to spit a stuck ball back out
    _eject_flywheel_velocity = -30.0

    # Seconds into an autonomous shot before the intake arm drops to the shooting position
    _auto_arm_down_delay_sec = 5.0

    @classmethod
    def create_shooter(
        cls,
        get_current_swerve_state,
        get_hub_center,
        launcher_offset_x: float,
        launcher_offset_y: float,
    ) -> Shooter:
        """
        Create a Shooter subsystem instance using the configured constant values.

        :param get_current_swerve_state: Function that returns the drivetrain state used by
            the shot solver.
        :type get_current_swerve_state: Callable[[], Any]
        :param get_hub_center: Function that returns the current hub center translation for the
            active alliance.
        :type get_hub_center: Callable[[], wpimath.geometry.Translation2d]
        :param launcher_offset_x: Forward launcher offset from the robot reference point in meters.
        :type launcher_offset_x: float
        :param launcher_offset_y: Lateral launcher offset from the robot reference point in meters.
        :type launcher_offset_y: float
        :returns: Configured shooter subsystem.
        :rtype: subsystems.shooter.Shooter
        """
        return Shooter(
            cls._canbus,
            cls._flywheel_motor_id,
            cls._flywheel_intake_motor_id,
            cls._flywheel_motor_configs,
            cls._flywheel_intake_motor_configs,
            cls._num_config_attempts,
            get_current_swerve_state,
            get_hub_center,
            launcher_offset_x,
            launcher_offset_y,
            cls._flywheel_target_offset_rps,
        )
