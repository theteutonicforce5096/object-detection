from bisect import bisect_left


class InterpolatingLookupTable:
    """
    Looks up values between measured points by interpolating linearly between them.

    Keys outside the measured range clamp to the nearest endpoint rather than extrapolating.
    """

    def __init__(self) -> None:
        """Create an empty interpolation table."""
        # Keys and values are kept in parallel sorted lists so bisect can find the surrounding
        # points in O(log n) time.
        self._keys: list[float] = []
        self._values: list[float] = []

    def __len__(self) -> int:
        """Return the number of calibration points stored in the table."""
        return len(self._keys)

    def put(self, key: float, value: float) -> None:
        """
        Insert or replace a calibration point in the sorted table.

        :param key: Lookup key to store, typically shot distance in meters.
        :type key: float
        :param value: Value associated with the lookup key.
        :type value: float
        """
        index = bisect_left(self._keys, key)
        if index < len(self._keys) and self._keys[index] == key:
            # Re-adding a distance replaces its value rather than storing a second point there.
            self._values[index] = value
            return

        self._keys.insert(index, key)
        self._values.insert(index, value)

    def get(self, key: float) -> float | None:
        """
        Return the interpolated value for a key, clamping outside the range.

        :param key: Lookup key to evaluate, typically shot distance in meters.
        :type key: float
        :returns: Interpolated value for the key, or None when the table is empty.
        :rtype: float | None
        """
        if not self._keys:
            return None

        index = bisect_left(self._keys, key)
        if index < len(self._keys) and self._keys[index] == key:
            return self._values[index]

        # Outside the measured range, hold the nearest endpoint instead of extrapolating.
        if index == 0:
            return self._values[0]
        if index >= len(self._keys):
            return self._values[-1]

        low_key = self._keys[index - 1]
        high_key = self._keys[index]
        low_value = self._values[index - 1]
        high_value = self._values[index]

        if high_key == low_key:
            return low_value

        # Blend linearly between the two surrounding points.
        blend = (key - low_key) / (high_key - low_key)
        return low_value + (high_value - low_value) * blend

    def clear(self) -> None:
        """Remove every stored calibration point."""
        self._keys.clear()
        self._values.clear()
