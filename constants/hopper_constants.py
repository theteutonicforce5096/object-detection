from phoenix6 import CANBus, signals
from phoenix6.configs import TalonFXConfiguration

from subsystems.hopper import Hopper


class HopperConstants:
    """Hardware IDs, motor configs, and tuned pulse timings used to build the hopper."""

    # CANBus instance
    _canbus = CANBus("Drivetrain")

    # CAN IDs
    _mecanum_wheel_id = 50
    _agitator_wheel_id = 51

    # Number of times to attempt to configure each device. Sized so retries span the several
    # seconds after code start when Phoenix license checks can reject configs on a pro bus.
    _num_config_attempts = 20

    # Mecanum Wheel Configs (TalonFX)
    _mecanum_wheel_configs = TalonFXConfiguration()
    _mecanum_wheel_configs.motor_output.with_neutral_mode(signals.NeutralModeValue.BRAKE)
    _mecanum_wheel_configs.current_limits.with_stator_current_limit(40)
    _mecanum_wheel_configs.current_limits.with_stator_current_limit_enable(True)
    # Supply cap bounds what a jammed feed can pull from the battery
    _mecanum_wheel_configs.current_limits.with_supply_current_limit(20)
    _mecanum_wheel_configs.current_limits.with_supply_current_limit_enable(True)
    _mecanum_wheel_configs.slot0.with_k_s(0.15736)
    _mecanum_wheel_configs.slot0.with_k_v(0.122)
    _mecanum_wheel_configs.slot0.with_k_p(0.5)
    _mecanum_wheel_configs.slot0.with_k_i(0)
    _mecanum_wheel_configs.slot0.with_k_d(0)

    # Agitator Wheel Configs (TalonFX)
    _agitator_wheel_configs = TalonFXConfiguration()
    _agitator_wheel_configs.motor_output.with_neutral_mode(signals.NeutralModeValue.COAST)
    # Breaking a fully packed hopper loose is a stall-torque problem, and torque comes from
    # stator current, so the limit stays high. At the ~3 V drive voltage (~25% duty) even a
    # hard stall at 80 A stator draws only about 20 A from the battery, which the supply cap
    # formalizes.
    _agitator_wheel_configs.current_limits.with_stator_current_limit(80)
    _agitator_wheel_configs.current_limits.with_stator_current_limit_enable(True)
    _agitator_wheel_configs.current_limits.with_supply_current_limit(20)
    _agitator_wheel_configs.current_limits.with_supply_current_limit_enable(True)

    # Feed pulse: net-forward push that drives balls toward the shooter.
    _feed_mecanum_velocity = 25.0
    _feed_agitator_volts = 3.0
    _feed_forward_sec = 0.5

    # Shake pulse: a brief reverse that breaks up balls bridged across a full hopper. Feeding
    # alternates the forward pulse with this one, and the forward pulse moves more, so the net
    # motion is still toward the shooter.
    _shake_mecanum_velocity = -15.0
    _shake_agitator_volts = -3.0
    _shake_reverse_sec = 0.125

    # Eject/unjam pulse: reverse the hopper to clear a stuck ball back out the intake.
    _eject_mecanum_velocity = -20.0
    _eject_agitator_volts = -3.0

    @classmethod
    def create_hopper(cls) -> Hopper:
        """
        Create a Hopper subsystem instance using the configured constant values.

        :returns: Configured hopper subsystem.
        :rtype: subsystems.hopper.Hopper
        """
        return Hopper(
            cls._canbus,
            cls._mecanum_wheel_id,
            cls._agitator_wheel_id,
            cls._mecanum_wheel_configs,
            cls._agitator_wheel_configs,
            cls._num_config_attempts,
            cls._feed_mecanum_velocity,
            cls._feed_agitator_volts,
            cls._feed_forward_sec,
            cls._shake_mecanum_velocity,
            cls._shake_agitator_volts,
            cls._shake_reverse_sec,
        )
