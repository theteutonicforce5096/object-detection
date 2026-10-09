from typing import Any

from phoenix6.status_code import StatusCode
from wpilib import Timer, reportWarning, wait


def configure_device(
    device: Any,
    configs: Any,
    num_attempts: int,
    *,
    per_attempt_timeout: float = 0.25,
    retry_delay: float = 0.25,
    max_wait: float = 10.0,
) -> StatusCode:
    """
    Apply a set of configs to a CTRE device once that device is on the bus.

    Waits for the device to appear on its CAN bus, then retries the config until the device
    reports OK or the time budget runs out. Each device is gated on its own connection, so
    devices on the CANivore and on the roboRIO bus are each configured as soon as they are
    ready. The retry budget also covers the several seconds after code start during which
    Phoenix license checks can reject configs on a pro bus ("Using Pro Feature on Unlicensed
    Device") even though every device is licensed. A device that never answers logs a warning
    instead of raising, so one bad device cannot stop the rest of the robot from starting.

    :param device: The CTRE device (motor controller, CANcoder, CANdle) to configure.
    :type device: Any
    :param configs: The configuration to apply to the device.
    :type configs: Any
    :param num_attempts: Number of times to attempt to configure the device.
    :type num_attempts: int
    :param per_attempt_timeout: Seconds each apply call waits for the device to respond.
    :type per_attempt_timeout: float
    :param retry_delay: Seconds to wait between attempts while the device comes up.
    :type retry_delay: float
    :param max_wait: Total seconds to spend waiting for the device before giving up.
    :type max_wait: float
    :returns: Final status code reported by the device, OK when the config applied.
    :rtype: phoenix6.status_code.StatusCode
    """
    # At least one attempt always runs, so a zero or negative count cannot silently skip the
    # config and leave the device on factory defaults.
    num_attempts = max(1, num_attempts)

    deadline = Timer.getFPGATimestamp() + max_wait

    # Devices on the CANivore ("Drivetrain") can take a few seconds to appear after the RIO
    # boots, so wait for the device to answer before configuring it.
    while not device.is_connected and Timer.getFPGATimestamp() < deadline:
        wait(retry_delay)

    # Retry until the device confirms OK or the budget runs out. The attempt count caps the
    # retries once the bus is up, so a bad config fails fast instead of burning the deadline.
    status_code: StatusCode = StatusCode.STATUS_CODE_NOT_INITIALIZED
    for _ in range(num_attempts):
        status_code = device.configurator.apply(configs, per_attempt_timeout)
        if status_code.is_ok():
            return status_code
        if Timer.getFPGATimestamp() >= deadline:
            break
        wait(retry_delay)

    reportWarning(
        f"Device with CAN ID {device.device_id} failed to config with error: {status_code.name}",
        False,
    )
    return status_code


def check_signal_status(status_code: StatusCode, context: str) -> None:
    """
    Warn when a CAN signal setup call fails instead of dropping the returned status.

    Bus optimization and update-frequency calls return a status code that is easy to ignore. A
    failure quietly leaves a signal at the wrong rate, degrading whatever control logic reads
    it, so the failure is surfaced as a driver station warning.

    :param status_code: Status returned by the signal setup call.
    :type status_code: phoenix6.status_code.StatusCode
    :param context: Description of the call for the warning message.
    :type context: str
    """
    if not status_code.is_ok():
        reportWarning(f"{context} failed with status: {status_code.name}", False)
