import phoenix6
import phoenix6.controls as controls
from commands2 import Subsystem
from phoenix6.hardware import CANdle
from phoenix6.signals import rgbw_color
from wpilib import SmartDashboard

from subsystems.device_config import configure_device


class LED(Subsystem):
    """
    Drives the CANdle LED strip that signals robot state to the drivers and crowd.

    Each method sets one animation on the strip and records a label for the dashboard. Every
    animation is exclusive, so the methods clear the strip before writing their own pattern.
    """

    # Number of CANdle animation slots cleared when turning the strip off
    _NUM_ANIMATION_SLOTS = 5

    def __init__(
        self,
        canbus: phoenix6.CANBus,
        led_id: int,
        led_configs: phoenix6.configs.CANdleConfiguration,
        num_config_attempts: int,
        led_end_index: int,
    ):
        """
        Initialize the LEDs using the specified constants.

        :param canbus: CANBus instance that electronics are on
        :type canbus: phoenix6.CANBus
        :param led_id: CAN ID of the LED controller
        :type led_id: int
        :param led_configs: Configs for the LED controller
        :type led_configs: phoenix6.configs.CANdleConfiguration
        :param num_config_attempts: Number of times to attempt to configure the device
        :type num_config_attempts: int
        :param led_end_index: End index of the animated LED range
        :type led_end_index: int
        """
        Subsystem.__init__(self)

        # Create LED
        self.candle = CANdle(led_id, canbus)

        # Apply LED configs
        configure_device(self.candle, led_configs, num_config_attempts)

        # Define LED end index
        self.led_end_index = led_end_index

        # Label for whatever animation is on the strip. It goes to the dashboard so the LEDs can
        # be explained without tracing back which binding fired.
        self.current_state = "Off"

    def periodic(self):
        """Publish the current LED state to the dashboard."""
        SmartDashboard.putString("LED/State", self.current_state)

    def auto_in_progress(self):
        """Show a solid green strip while autonomous is running."""
        self.extinguish()
        animation_control = controls.SolidColor(
            0, self.led_end_index, rgbw_color.RGBWColor(53, 147, 87, 0)
        )
        self.candle.set_control(animation_control)
        self.current_state = "Auto In Progress"

    def hopper_full(self):
        """Play a yellow sweep to signal that the hopper is full."""
        self.extinguish()
        animation_control = controls.LarsonAnimation(
            0, self.led_end_index, 0, rgbw_color.RGBWColor(225, 242, 0, 0)
        )
        self.candle.set_control(animation_control)
        self.current_state = "Hopper Full"

    def shooting_manual(self):
        """Play a fire animation while a manual shot is in progress."""
        self.extinguish()
        animation_control = controls.FireAnimation(
            0,
            self.led_end_index,
            1,
            1,
            phoenix6.signals.spn_enums.AnimationDirectionValue.FORWARD,
            0.6,
            0.3,
            60,
        )
        self.candle.set_control(animation_control)
        self.current_state = "Shooting Manual"

    def shooting_calculated(self):
        """Play a blue sweep while a distance-based shot is in progress."""
        self.extinguish()
        animation_control = controls.LarsonAnimation(
            0, self.led_end_index, 0, rgbw_color.RGBWColor(40, 60, 255, 0)
        )
        self.candle.set_control(animation_control)
        self.current_state = "Shooting Calculated"

    def default(self):
        """Play the idle animation the robot shows whenever nothing else is happening."""
        self.extinguish()
        animation_control = controls.StrobeAnimation(
            0, self.led_end_index, 4, rgbw_color.RGBWColor(102, 225, 0, 0), 4
        )
        self.candle.set_control(animation_control)
        self.current_state = "Default"

    def pride(self):
        """Play a rainbow animation."""
        self.extinguish()
        animation_control = controls.RainbowAnimation(
            0,
            self.led_end_index,
            3,
            1,
            phoenix6.signals.spn_enums.AnimationDirectionValue.FORWARD,
            100,
        )
        self.candle.set_control(animation_control)
        self.current_state = "Pride"

    def five_seconds_left(self):
        """Play a fast red sweep to warn that five seconds are left in the match."""
        self.extinguish()
        animation_control = controls.LarsonAnimation(
            0,
            self.led_end_index,
            2,
            rgbw_color.RGBWColor(225, 0, 0, 0),
            3,
            phoenix6.signals.spn_enums.LarsonBounceValue.FRONT,
            25,
        )
        self.candle.set_control(animation_control)
        self.current_state = "Five Seconds Left"

    def extinguish(self):
        """Clear every animation slot so the strip goes dark."""
        for animation_slot in range(self._NUM_ANIMATION_SLOTS):
            animation_control = controls.EmptyAnimation(animation_slot)
            self.candle.set_control(animation_control)

        # The other animation methods clear the strip before setting their own pattern, so this
        # label is only what the dashboard shows when extinguish is called on its own.
        self.current_state = "Off"
