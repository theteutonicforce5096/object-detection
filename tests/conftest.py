"""Shared pytest configuration for the pyfrc test suite."""

import contextlib
import threading

import pyfrc.test_support.controller as pyfrc_controller

# pyfrc hardcodes short startup timeouts (1 s for the robot thread, 2 s for robotInit) with no
# configuration hook. robotInit builds every subsystem and all of the Phoenix config objects,
# which can legitimately exceed 2 s on a slow development machine and fail the deploy-time test
# run. Scaling the timeouts keeps the checks (a hung robotInit still fails) without punishing
# slow hardware; fast machines are unaffected because the waits return as soon as startup
# completes.
_STARTUP_TIMEOUT_SCALE = 15

_original_run_robot = pyfrc_controller.TestController.run_robot
_original_wait_for = threading.Condition.wait_for


def _scaled_wait_for(self, predicate, timeout=None):
    if timeout is not None:
        timeout = timeout * _STARTUP_TIMEOUT_SCALE
    return _original_wait_for(self, predicate, timeout)


@contextlib.contextmanager
def _run_robot_with_patient_startup(self):
    with contextlib.ExitStack() as stack:
        # Only startup (entering the original context manager) runs with scaled timeouts; the
        # patch is removed before the test body executes.
        threading.Condition.wait_for = _scaled_wait_for
        try:
            stack.enter_context(_original_run_robot(self))
        finally:
            threading.Condition.wait_for = _original_wait_for
        yield


pyfrc_controller.TestController.run_robot = _run_robot_with_patient_startup
