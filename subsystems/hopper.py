from commands2 import Command, Subsystem
from commands2.sysid import SysIdRoutine
from phoenix6 import CANBus, SignalLogger
from phoenix6.configs import TalonFXConfiguration
from phoenix6.controls import VelocityVoltage, VoltageOut
from phoenix6.hardware import TalonFX
from wpilib import RobotBase, SmartDashboard
from wpilib.sysid import SysIdRoutineLog

from subsystems.device_config import configure_device


class Hopper(Subsystem):
    """
    Moves balls from the hopper into the shooter.

    A mecanum wheel carries balls toward the shooter under closed-loop velocity control, and an
    agitator wheel stirs the stack so balls do not bridge over each other on the way out.
    """

    def __init__(
        self,
        canbus: CANBus,
        mecanum_wheel_id: int,
        agitator_wheel_id: int,
        mecanum_wheel_configs: TalonFXConfiguration,
        agitator_wheel_configs: TalonFXConfiguration,
        num_config_attempts: int,
        feed_mecanum_velocity: float,
        feed_agitator_volts: float,
        feed_forward_sec: float,
        shake_mecanum_velocity: float,
        shake_agitator_volts: float,
        shake_reverse_sec: float,
    ):
        """
        Initialize the hopper using the specified constants.

        :param canbus: CANBus instance that electronics are on
        :type canbus: phoenix6.CANBus
        :param mecanum_wheel_id: CAN ID of the mecanum wheel
        :type mecanum_wheel_id: int
        :param agitator_wheel_id: CAN ID of the agitator wheel
        :type agitator_wheel_id: int
        :param mecanum_wheel_configs: Configs for the mecanum wheel
        :type mecanum_wheel_configs: phoenix6.configs.TalonFXConfiguration
        :param agitator_wheel_configs: Configs for the agitator wheel
        :type agitator_wheel_configs: phoenix6.configs.TalonFXConfiguration
        :param num_config_attempts: Number of times to attempt to configure each device
        :type num_config_attempts: int
        :param feed_mecanum_velocity: Mecanum-wheel velocity in rotations per second during the
            forward feed pulse.
        :type feed_mecanum_velocity: float
        :param feed_agitator_volts: Agitator voltage during the forward feed pulse.
        :type feed_agitator_volts: float
        :param feed_forward_sec: Duration in seconds of the forward feed pulse.
        :type feed_forward_sec: float
        :param shake_mecanum_velocity: Mecanum-wheel velocity in rotations per second during the
            reverse shake pulse.
        :type shake_mecanum_velocity: float
        :param shake_agitator_volts: Agitator voltage during the reverse shake pulse.
        :type shake_agitator_volts: float
        :param shake_reverse_sec: Duration in seconds of the reverse shake pulse.
        :type shake_reverse_sec: float
        """
        Subsystem.__init__(self)

        # Store feed/shake oscillation tunables
        self.feed_mecanum_velocity = feed_mecanum_velocity
        self.feed_agitator_volts = feed_agitator_volts
        self.feed_forward_sec = feed_forward_sec
        self.shake_mecanum_velocity = shake_mecanum_velocity
        self.shake_agitator_volts = shake_agitator_volts
        self.shake_reverse_sec = shake_reverse_sec

        # Create motors
        self.mecanum_wheel = TalonFX(mecanum_wheel_id, canbus)
        self.agitator_wheel = TalonFX(agitator_wheel_id, canbus)

        # Apply motor configs
        configure_device(self.mecanum_wheel, mecanum_wheel_configs, num_config_attempts)
        configure_device(self.agitator_wheel, agitator_wheel_configs, num_config_attempts)

        if not RobotBase.isSimulation():
            # Nothing here reads the motors, so every signal stays trimmed except the two
            # velocities the dashboard reports. Those are how the drivers confirm the hopper is
            # actually turning during a feed.
            self.mecanum_wheel.optimize_bus_utilization()
            self.agitator_wheel.optimize_bus_utilization()
            self.mecanum_wheel.get_velocity().set_update_frequency(50.0)
            self.agitator_wheel.get_velocity().set_update_frequency(50.0)

        # Create control requests
        self.velocity_pid_request = VelocityVoltage(velocity=0)
        self.voltage_request = VoltageOut(output=0)

        # Create SysId routine for characterizing the mecanum wheel motor
        def record_mecanum_state(state):
            SignalLogger.write_string(
                "SysId_Hopper_Mecanum_Motor_State", SysIdRoutineLog.stateEnumToString(state)
            )

        def drive_mecanum(output):
            self.mecanum_wheel.set_control(self.voltage_request.with_output(output))

        self.mecanum_motor_sys_id_routine = SysIdRoutine(
            SysIdRoutine.Config(
                rampRate=0.5,
                stepVoltage=9.0,
                timeout=15.0,
                recordState=record_mecanum_state,
            ),
            SysIdRoutine.Mechanism(
                drive_mecanum,
                lambda log: None,
                self,
            ),
        )

        # Routines this subsystem can characterize. RobotContainer gathers these, along with the
        # other subsystems' routines, into the single SysId chooser on the dashboard.
        self.sys_id_routines = [("Mecanum", self.mecanum_motor_sys_id_routine)]

    def periodic(self):
        """Publish the hopper's wheel velocities to the dashboard."""
        SmartDashboard.putNumber(
            "Hopper/Mecanum Velocity (rps)", self.mecanum_wheel.get_velocity().value_as_double
        )
        SmartDashboard.putNumber(
            "Hopper/Agitator Velocity (rps)", self.agitator_wheel.get_velocity().value_as_double
        )

    def run_hopper(self, mecanum_velocity, agitator_volts):
        """
        Build a one-shot command that applies the requested hopper outputs.

        :param mecanum_velocity: Desired closed-loop velocity for the mecanum wheel in rotations
            per second.
        :type mecanum_velocity: float
        :param agitator_volts: Desired open-loop voltage for the agitator wheel.
        :type agitator_volts: float
        :returns: Command that applies the requested hopper outputs once.
        :rtype: commands2.Command
        """
        return self.runOnce(lambda: self.set_hopper_speeds(mecanum_velocity, agitator_volts))

    def set_hopper_speeds(self, mecanum_velocity, agitator_volts):
        """
        Apply the requested mecanum-wheel velocity and agitator voltage.

        :param mecanum_velocity: Desired closed-loop velocity for the mecanum wheel in rotations
            per second.
        :type mecanum_velocity: float
        :param agitator_volts: Desired open-loop voltage for the agitator wheel.
        :type agitator_volts: float
        """
        self.mecanum_wheel.set_control(self.velocity_pid_request.with_velocity(mecanum_velocity))
        self.agitator_wheel.set_control(self.voltage_request.with_output(agitator_volts))

    def _drive_hopper_for(self, mecanum_velocity, agitator_volts, seconds) -> Command:
        """
        Build a command that holds the given hopper outputs for a fixed duration.

        :param mecanum_velocity: Mecanum-wheel velocity in rotations per second to hold.
        :type mecanum_velocity: float
        :param agitator_volts: Agitator voltage to hold.
        :type agitator_volts: float
        :param seconds: Duration in seconds to hold the outputs before the command ends.
        :type seconds: float
        :returns: Command that applies the outputs each loop until the timeout elapses.
        :rtype: commands2.Command
        """
        return self.run(
            lambda: self.set_hopper_speeds(mecanum_velocity, agitator_volts)
        ).withTimeout(seconds)

    def create_feed_cycle_command(self) -> Command:
        """
        Build one feed oscillation: push balls toward the shooter, then shake back briefly.

        The forward pulse drives balls into the shooter, and the shorter reverse pulse jostles a
        packed hopper so bridged balls break loose instead of stalling the wheels. The forward
        pulse moves more than the shake pulse, so the net motion is toward the shooter. The
        command covers a single oscillation, which lets the caller re-check whether the flywheel
        is still at speed before feeding again.

        :returns: Command that runs one forward-then-shake feed oscillation.
        :rtype: commands2.Command
        """
        return self._drive_hopper_for(
            self.feed_mecanum_velocity, self.feed_agitator_volts, self.feed_forward_sec
        ).andThen(
            self._drive_hopper_for(
                self.shake_mecanum_velocity, self.shake_agitator_volts, self.shake_reverse_sec
            )
        )

    def create_stop_command(self) -> Command:
        """
        Build a one-shot command that stops both hopper motors.

        :returns: Command that zeroes both hopper outputs.
        :rtype: commands2.Command
        """
        return self.run_hopper(0, 0)
