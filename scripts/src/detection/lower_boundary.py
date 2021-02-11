import numpy as np


class LowerBoundary:
    def __init__(self):
        self.lower_boundaries = {"purple": [117, 20, 94], "white": [33, 15, 160],
                                 "yellow": [24, 190, 127],
                                 "blue": [101, 50, 0], "orange": [0, 196, 123],
                                 "red": [0, 216, 92],
                                 "brown": [0, 121, 15], "green": [47, 89, 0],
                                 "black": [31, 21, 0],
                                 "grey": [22, 6, 54], "obstacle" : [28, 169, 74]}

    def get_lower_boundaries(self, color_to_detect):
        if color_to_detect in self.lower_boundaries:
            return np.array(self.lower_boundaries[color_to_detect])
        return np.zeros(3)