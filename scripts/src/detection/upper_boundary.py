import numpy as np


class UpperBoundary:

    def __init__(self):
        self.upper_boundaries = {"purple": [143, 255, 233], "white": [62, 50, 255],
                                 "yellow": [79, 255, 228],
                                 "blue": [179, 255, 255], "orange": [16, 255, 255],
                                 "red": [179, 255, 106],
                                 "brown": [ 26, 255, 39], "green": [75, 255, 255],
                                 "black": [126, 255,17],
                                 "grey": [70, 113,74]}

    def get_upper_boundaries(self, color_to_detect):
        if color_to_detect in self.upper_boundaries:
            return np.array(self.upper_boundaries[color_to_detect])
        else:
            return np.zeros(3)