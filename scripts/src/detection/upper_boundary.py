import numpy as np

from scripts.src.mapping.color import Color


class UpperBoundary:
    """Upper boundaries of colors of pucks"""
    def __init__(self):
        self.upper_boundaries = {Color.VIOLET: [143, 255, 233], Color.WHITE: [62, 50, 255],
                                 Color.YELLOW: [79, 255, 228],
                                 Color.BLUE: [179, 255, 255], Color.ORANGE: [16, 255, 255],
                                 Color.RED: [179, 255, 106],
                                 Color.BROWN: [26, 255, 39], Color.GREEN: [75, 255, 255],
                                 Color.BLACK: [126, 255,17],
                                 Color.GREY: [70, 113, 74], "square": [104, 255, 125]}

    def get_upper_boundaries(self, color_to_detect):
        if color_to_detect in self.upper_boundaries:
            return np.array(self.upper_boundaries[color_to_detect])
        return np.zeros(3)
