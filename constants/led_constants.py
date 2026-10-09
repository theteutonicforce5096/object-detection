from phoenix6 import CANBus
from phoenix6.configs import CANdleConfiguration
from phoenix6.signals import StripTypeValue

from subsystems.led import LED


class LEDConstants:
    """Hardware IDs and configs used to build the LED subsystem."""

    # The CANdle is wired to the roboRIO CAN bus, not the CANivore the rest of the robot uses.
    _canbus = CANBus("rio")

    # CAN IDs
    _led_id = 3

    # Number of times to attempt to configure each device. Sized so retries span the several
    # seconds after code start when Phoenix license checks can reject configs on a pro bus.
    _num_config_attempts = 20

    # LED Configs (CANdle)
    _led_configs = CANdleConfiguration()
    _led_configs.led.with_strip_type(StripTypeValue.GRB)

    # Number of LEDs on the strip
    _led_end_index = 64

    @classmethod
    def create_led(cls) -> LED:
        """
        Create an LED subsystem instance using the configured constant values.

        :returns: Configured LED subsystem.
        :rtype: subsystems.led.LED
        """
        return LED(
            cls._canbus, cls._led_id, cls._led_configs, cls._num_config_attempts, cls._led_end_index
        )
