from math import pi
from typing import Callable

from phoenix6 import swerve
from wpimath.geometry import Pose2d

from constants.swerve_drivetrain_constants import SwerveDrivetrainConstants
from subsystems.vision import Vision


class VisionConstants:
    """Trust and rejection thresholds used to build the vision subsystem."""

    # Baseline measurement uncertainty before any scaling. The angular baseline is deliberately
    # enormous so odometry keeps its Pigeon heading and ignores the vision heading.
    _linear_std_dev_baseline = 1.5
    _angular_std_dev_baseline = 1000000.0 * pi

    # Per-camera multipliers on the standard deviations, in camera order
    # (back_camera, front_left_camera, front_right_camera). Raise one to trust that camera less.
    _camera_std_dev_factors = (1.0, 1.0, 1.0)

    # Speeds past which vision measurements are trusted less, as a fraction of the drivetrain
    # maximums. Both are low because a moving robot blurs the tags.
    _max_linear_speed = SwerveDrivetrainConstants._max_linear_speed * 0.25
    _max_angular_speed = SwerveDrivetrainConstants._max_angular_speed * 0.20

    # Tilt in degrees past which measurements are thrown out, since a tipping robot breaks the
    # flat-on-the-floor assumption the pose solve relies on
    _max_tilt_deg = 5.0

    @classmethod
    def create_vision(
        cls,
        add_vision_measurement,
        get_current_swerve_state: Callable[[], swerve.SwerveDrivetrain.SwerveDriveState],
        get_robot_tilt: Callable[[], tuple[float, float]],
        set_camera_pose: Callable[[str, Pose2d], None],
    ) -> Vision:
        """
        Create a Vision subsystem instance using the configured constant values.

        :param add_vision_measurement: Callback used to inject accepted vision measurements into
            drivetrain odometry.
        :type add_vision_measurement:
            Callable[[wpimath.geometry.Pose2d, float, tuple[float, float, float]], None]
        :param get_current_swerve_state: Function that returns the current drivetrain state.
        :type get_current_swerve_state:
            Callable[[], phoenix6.swerve.SwerveDrivetrain.SwerveDriveState]
        :param get_robot_tilt: Function that returns the current robot pitch and roll in degrees.
        :type get_robot_tilt: Callable[[], tuple[float, float]]
        :param set_camera_pose: Callback used to show a camera's latest accepted estimate on the
            drivetrain's field widget.
        :type set_camera_pose: Callable[[str, wpimath.geometry.Pose2d], None]
        :returns: Configured vision subsystem.
        :rtype: subsystems.vision.Vision
        """
        return Vision(
            add_vision_measurement,
            get_current_swerve_state,
            get_robot_tilt,
            set_camera_pose,
            SwerveDrivetrainConstants._field_type,
            cls._linear_std_dev_baseline,
            cls._angular_std_dev_baseline,
            cls._camera_std_dev_factors,
            cls._max_linear_speed,
            cls._max_angular_speed,
            cls._max_tilt_deg,
        )
