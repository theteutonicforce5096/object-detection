import csv
from pathlib import Path
from typing import Final

from constants.interpolating_lookup_table import InterpolatingLookupTable
from constants.shot_calculator_constants import calc_shot_profile

# Measured shot table, stored as distance_m, flywheel_rps, time_of_flight_sec rows. Speeds came
# from 240 fps video of test shots, with distances measured to the hub center. A simulator fit
# to those shots (slip_factor=0.448, drag_coeff=0.104, magnus_coeff=-0.4, launch_angle_deg=67.5)
# filled in the gaps between them. load_lut_entry turns these absolute targets into corrections.
_CALIBRATION_TABLE_PATH: Final[Path] = (
    Path(__file__).resolve().parent.parent / "constants" / "shot_calibration_table.csv"
)


class ShotCalculator:
    """
    Works out the flywheel speed needed to reach the hub from a given distance.

    A projectile equation gives a baseline speed, and a table of measured shots supplies the
    correction from that baseline to what the real shooter needs. Splitting it this way means
    the calculator still extrapolates sensibly past the calibrated range instead of flattening
    out at the last table entry.
    """

    def __init__(self) -> None:
        """Create a shot calculator and load the default calibration residuals."""
        # Corrections on top of the physics baseline, keyed by shooting distance in meters. The
        # calibration entries also carry a time of flight, which is kept as data but unused here.
        self._rps_residual_map = InterpolatingLookupTable()

        with _CALIBRATION_TABLE_PATH.open(newline="") as calibration_file:
            for row in csv.DictReader(calibration_file):
                self.load_lut_entry(float(row["distance_m"]), float(row["flywheel_rps"]))

    def _lookup_required(
        self, table: InterpolatingLookupTable, distance: float, label: str
    ) -> float:
        """
        Read a correction from a lookup table, raising if the table is empty.

        An empty table means the calibration data never loaded, which would otherwise show up as
        silently wrong shots rather than an error.

        :param table: Correction lookup table to read from.
        :type table: constants.interpolating_lookup_table.InterpolatingLookupTable
        :param distance: Shooting distance in meters used as the lookup key.
        :type distance: float
        :param label: Human-readable name of the lookup table for error reporting.
        :type label: str
        :returns: Correction value stored in the lookup table.
        :rtype: float
        """
        value = table.get(distance)
        if value is None:
            raise ValueError(f"{label} lookup table is empty")
        return value

    def _physics_rps(self, distance: float) -> float:
        """
        Get the baseline flywheel speed from the closed-form shot model.

        :param distance: Shooting distance in meters.
        :type distance: float
        :returns: Baseline flywheel target in rotations per second.
        :rtype: float
        """
        return calc_shot_profile(distance)

    def load_lut_entry(self, distance_m: float, flywheel_rps: float) -> None:
        """
        Store one measured shot as a correction on top of the physics model.

        :param distance_m: Shooting distance in meters represented by the table entry.
        :type distance_m: float
        :param flywheel_rps: Measured flywheel target in rotations per second.
        :type flywheel_rps: float
        """
        # Entries arrive as absolute targets but are stored as the gap from the physics
        # baseline, which is what lets the two be added back together at lookup time.
        physics_rps = self._physics_rps(distance_m)

        self._rps_residual_map.put(distance_m, flywheel_rps - physics_rps)

    def _effective_rps(self, distance: float) -> float:
        """
        Add the measured correction to the physics baseline for a given distance.

        :param distance: Shooting distance in meters.
        :type distance: float
        :returns: Calibrated flywheel target in rotations per second.
        :rtype: float
        """
        return self._physics_rps(distance) + self._lookup_required(
            self._rps_residual_map,
            distance,
            "RPS residual",
        )

    def get_profile_for_distance(self, distance: float) -> float:
        """
        Get the flywheel speed to shoot from a given distance.

        :param distance: Shooting distance in meters.
        :type distance: float
        :returns: Calibrated flywheel target in rotations per second.
        :rtype: float
        """
        return self._effective_rps(distance)
