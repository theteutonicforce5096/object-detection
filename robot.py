import commands2
from phoenix6 import SignalLogger
from wpilib import DriverStation, SmartDashboard, getDeployDirectory
from wpinet import WebServer

from robot_container import RobotContainer


class RebuiltRobot(commands2.TimedCommandRobot):
    """Top-level robot class for Team 5096's 2026 Rebuilt robot."""

    def robotInit(self):
        """Set up logging, the dashboard web server, and the robot container."""
        # Phoenix 6 logs every signal on the bus by default, which is unnecessary.
        # Match logging is started by hand in autonomousInit only when an FMS is
        # attached, since the FMS is not attached yet when the robot boots in the pit.
        SignalLogger.enable_auto_logging(False)

        # Serve the deploy folder so Elastic's "Load Layout From Robot" can pull the dashboard
        # layout straight off the robot. Every driver station then shows the same tabs without
        # anyone copying a file around.
        WebServer.getInstance().start(5800, getDeployDirectory())

        # Building the container also configures every CAN device, which waits for each device
        # to appear on its bus before applying configs (see configure_device).
        self.robot_container = RobotContainer()

    def robotPeriodic(self):
        """Run the command scheduler and publish the match countdown to the dashboard."""
        super().robotPeriodic()

        # The dashboard's match timer reads this key. Nothing else publishes the match clock
        # to NetworkTables.
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime())

    def autonomousInit(self):
        """Start match logging, clear leftover commands, and schedule the selected auto."""
        # The FMS is attached by the time the match starts, so this is where match logging can
        # actually begin (robotInit runs in the pit, long before the FMS connects).
        if DriverStation.isFMSAttached():
            SignalLogger.start()

        commands2.CommandScheduler.getInstance().cancelAll()
        self.robot_container.create_commands_auto()
        self.autonomous_command = self.robot_container.get_selected_auto_command()
        if self.autonomous_command is not None:
            self.autonomous_command.schedule()

    def teleopInit(self):
        """Clear leftover autonomous commands and prepare the robot for driver control."""
        commands2.CommandScheduler.getInstance().cancelAll()
        self.robot_container.create_commands_teleop()

    def teleopExit(self):
        """Stop match logging once teleop ends during an FMS match."""
        if DriverStation.isFMSAttached():
            SignalLogger.stop()

    def testInit(self):
        """Clear leftover commands and put the robot into a safe state for test mode."""
        commands2.CommandScheduler.getInstance().cancelAll()
        self.robot_container.create_commands_test()
