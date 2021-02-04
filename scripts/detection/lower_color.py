import numpy as np
import color


class LowerBoundary:

    def get_lower(self, color_to_detect):
        if color_to_detect == color.Color.red.value:
            return self._get_red_lower()
        elif color_to_detect == color.Color.blue.value:
            return self._get_blue_lower()
        elif color_to_detect == color.Color.green.value:
            return self._get_green_lower()

    def _get_red_lower(self):
        return np.array([136, 87, 111], np.uint8)

    def _get_green_lower(self):
        return np.array([25, 52, 72], np.uint8)

    def _get_blue_lower(self):
        return np.array([94, 80, 2], np.uint8)
