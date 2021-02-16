import numpy as np

from scripts.src.mapping.color import Color


class LowerBoundary:
    """Lower boundaries of colors of pucks"""
    def __init__(self):
        self.lower_boundaries = {Color.VIOLET: [117, 20, 94], Color.WHITE: [33, 15, 160],
                                 Color.YELLOW: [24, 190, 127],
                                 Color.BLUE: [101, 50, 0], Color.ORANGE: [0, 196, 123],
                                 Color.RED: [0, 216, 92],
                                 Color.BROWN: [0, 121, 15], Color.GREEN: [47, 89, 0],
                                 Color.BLACK: [31, 21, 0],
                                 Color.GREY: [22, 6, 54]}

    def get_lower_boundaries(self, color_to_detect):
        if color_to_detect in self.lower_boundaries:
            return np.array(self.lower_boundaries[color_to_detect])
        return np.zeros(3)
