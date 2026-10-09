from math import cos, pi, radians, sqrt, tan

from wpilib import DriverStation
from wpimath.geometry import Pose2d, Rotation2d, Transform2d, Translation2d
from wpimath.units import inchesToMeters

# Shot geometry. The ball has to climb from the flywheel's height to the hub's, so the vertical
# distance is the difference between the two.
default_y_dis = inchesToMeters(72 - 21.25)  # 72 in. hub height - 21.25 in. flywheel height
default_r = inchesToMeters(3.95 / 2)  # Flywheel radius in meters
default_g = -9.8  # Gravity in meters per second squared
default_theta_deg = 67.5  # Fixed launch angle above horizontal, in degrees

# Field variants differ by about half an inch in hub position, which matters at long range.
default_field_type = "AndyMark"
default_shooter_offset = Transform2d(
    Translation2d(inchesToMeters(-7.78), inchesToMeters(-7.95)),
    Rotation2d(),
)

_hub_positions = {
    ("AndyMark", DriverStation.Alliance.kBlue): Translation2d(
        inchesToMeters(181.56),
        inchesToMeters(158.32),
    ),
    ("AndyMark", DriverStation.Alliance.kRed): Translation2d(
        inchesToMeters(468.56),
        inchesToMeters(158.32),
    ),
    ("Welded", DriverStation.Alliance.kBlue): Translation2d(
        inchesToMeters(182.11),
        inchesToMeters(158.84),
    ),
    ("Welded", DriverStation.Alliance.kRed): Translation2d(
        inchesToMeters(469.11),
        inchesToMeters(158.84),
    ),
}

# The robot reference point sits this far from the hub center, along x toward the alliance
# wall, when the robot is parked against the hub for an odometry reset. Surveyed on the Welded
# field; applying the same offset to each variant's hub center keeps the reset pose consistent
# on both field types.
_hub_reset_distance_m = 1.044


def _get_default_alliance_color():
    """
    Resolve the current alliance, defaulting to blue when it is unavailable.

    The alliance is unknown until the driver station connects, and blue keeps the field geometry
    valid until then.
    """
    alliance_color = DriverStation.getAlliance()
    if alliance_color == DriverStation.Alliance.kRed:
        return DriverStation.Alliance.kRed
    return DriverStation.Alliance.kBlue


def get_hub_center(
    field_type: str = default_field_type,
    alliance_color: DriverStation.Alliance | None = None,
):
    """
    Get the hub center translation for the current field setup.

    :param field_type: Active field variant name used to choose the hub coordinates.
    :type field_type: str
    :param alliance_color: Alliance used to select the correct hub position, or None to
        infer it.
    :type alliance_color: wpilib.DriverStation.Alliance | None
    :returns: Hub center translation for the requested field and alliance.
    :rtype: wpimath.geometry.Translation2d
    """
    resolved_alliance = (
        alliance_color if alliance_color is not None else _get_default_alliance_color()
    )
    hub_center = _hub_positions.get((field_type, resolved_alliance))
    if hub_center is None:
        raise ValueError(f"Unsupported field configuration: {field_type}, {resolved_alliance}")

    return hub_center


def get_hub_reset_pose(
    field_type: str = default_field_type,
    alliance_color: DriverStation.Alliance | None = None,
):
    """
    Get the hub-facing reference pose for re-seeding odometry.

    The pose is derived from the hub center for the active field variant, so a Welded and an
    AndyMark field each re-seed against their own hub position.

    :param field_type: Active field variant name used to choose the hub coordinates.
    :type field_type: str
    :param alliance_color: Alliance used to select the correct reset pose, or None to infer it.
    :type alliance_color: wpilib.DriverStation.Alliance | None
    :returns: Hub-facing reference pose for the requested alliance.
    :rtype: wpimath.geometry.Pose2d
    """
    resolved_alliance = (
        alliance_color if alliance_color is not None else _get_default_alliance_color()
    )
    hub_center = get_hub_center(field_type, resolved_alliance)

    # The robot parks on the alliance-wall side of its hub, facing it, so red sits at +x facing
    # 0 degrees and blue sits at -x facing 180 degrees.
    if resolved_alliance == DriverStation.Alliance.kRed:
        return Pose2d(hub_center.x + _hub_reset_distance_m, hub_center.y, Rotation2d.fromDegrees(0))
    return Pose2d(hub_center.x - _hub_reset_distance_m, hub_center.y, Rotation2d.fromDegrees(180))


def calc_shooter_to_hub_distance(
    robot_pose: Pose2d,
    hub_center: Translation2d,
    shooter_offset: Transform2d = default_shooter_offset,
):
    """
    Measure from the shooter to the hub, rather than from the robot's center.

    The shooter is mounted off-center, so using the robot's pose directly would be off by that
    offset and would bias every distance-based shot.

    :param robot_pose: Current robot pose on the field.
    :type robot_pose: wpimath.geometry.Pose2d
    :param hub_center: Field translation of the hub center.
    :type hub_center: wpimath.geometry.Translation2d
    :param shooter_offset: Transform from robot reference point to shooter exit point.
    :type shooter_offset: wpimath.geometry.Transform2d
    :returns: Shooter-to-hub distance in meters.
    :rtype: float
    """
    shooter_pose = robot_pose.transformBy(shooter_offset)
    shooter_to_hub = hub_center - shooter_pose.translation()
    return shooter_to_hub.norm()


def calc_velocity(
    x_dis: float,
    y_dis: float = default_y_dis,
    theta_deg: float = default_theta_deg,
    g: float = default_g,
):
    """
    Solve the launch speed a ball needs to land in the hub, ignoring air resistance.

    This is the standard projectile equation rearranged for speed, given that the launch angle is
    fixed by the hardware. Air resistance is left to the measured corrections in ShotCalculator.

    :param x_dis: Horizontal distance in meters from the shooter to the target.
    :type x_dis: float
    :param y_dis: Vertical distance in meters from the shooter to the target.
    :type y_dis: float
    :param theta_deg: Angle in degrees above horizontal that the ball leaves at.
    :type theta_deg: float
    :param g: Gravitational acceleration in meters per second squared.
    :type g: float
    :returns: Required launch speed in meters per second.
    :rtype: float
    """
    gravity_magnitude = abs(g)
    theta_rad = radians(theta_deg)
    denominator = 2 * (cos(theta_rad) ** 2) * (x_dis * tan(theta_rad) - y_dis)

    # A non-positive denominator means the target sits at or above the arc's peak, so no launch
    # speed at this angle can reach it.
    if denominator <= 0:
        raise ValueError("Shot is unreachable with the current launch angle and target height")

    ideal_velocity_mps = sqrt((gravity_magnitude * (x_dis**2)) / denominator)
    return ideal_velocity_mps


def shoot_speed(ideal_velocity_mps, r: float = default_r):
    """
    Convert a ball launch speed into the flywheel speed that produces it.

    Assumes the ball leaves at the flywheel's surface speed, which overstates it slightly since
    the ball slips against the wheel. The measured corrections in ShotCalculator absorb that gap.

    :param ideal_velocity_mps: Required ball launch speed in meters per second.
    :type ideal_velocity_mps: float
    :param r: Flywheel radius in meters.
    :type r: float
    :returns: Flywheel speed in rotations per second.
    :rtype: float
    """
    target_velocity = ideal_velocity_mps / (2 * pi * r)
    return target_velocity


def calc_shot_profile(
    x_dis: float,
    y_dis: float = default_y_dis,
    theta_deg: float = default_theta_deg,
    g: float = default_g,
    r: float = default_r,
):
    """
    Get the physics-only flywheel target for a given distance.

    This is the baseline ShotCalculator corrects with measured data, not a target to command
    directly.

    :param x_dis: Horizontal shooter-to-target distance in meters.
    :type x_dis: float
    :param y_dis: Vertical distance from shooter exit to target entry in meters.
    :type y_dis: float
    :param theta_deg: Launch angle in degrees above horizontal.
    :type theta_deg: float
    :param g: Gravitational acceleration in meters per second squared.
    :type g: float
    :param r: Flywheel radius in meters.
    :type r: float
    :returns: Target flywheel speed in rotations per second.
    :rtype: float
    """
    ideal_velocity_mps = calc_velocity(x_dis, y_dis, theta_deg, g)
    return shoot_speed(ideal_velocity_mps, r)


__all__ = [
    "calc_shot_profile",
    "calc_shooter_to_hub_distance",
    "calc_velocity",
    "default_field_type",
    "default_g",
    "default_r",
    "default_shooter_offset",
    "default_y_dis",
    "default_theta_deg",
    "get_hub_center",
    "get_hub_reset_pose",
    "shoot_speed",
]
