import numpy as np
import color


class UpperBoundary:

    def get_upper_boundary(self, color_to_detect):
        if color_to_detect == color.Color.red.value:
            return self._get_red_upper()
        elif color_to_detect == color.Color.blue.value:
            return self._get_blue_upper()
        elif color_to_detect == color.Color.green.value:
            return self._get_green_upper()

    def _get_red_upper(self):
        return np.array([180, 255, 255], np.uint8)

    def _get_green_upper(self):
        return np.array([102, 255, 255], np.uint8)

    def _get_blue_upper(self):
        return np.array([120, 255, 255], np.uint8)
