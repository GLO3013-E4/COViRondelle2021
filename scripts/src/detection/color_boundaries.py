import numpy as np


class ColorBoundaries:
    """Upper boundaries of colors of pucks"""
    def __init__(self):
        self.color_boundaries = {"purple": {"lower": [117, 20, 94], "upper":[143, 255, 233]},
                                 "white": {"lower": [20, 0, 155], "upper":[124, 62, 186]} ,
                                 "yellow": {"lower": [19, 105, 40], "upper":[41, 255, 255]},
                                 "blue": {"lower": [101, 50, 0], "upper":[179, 255, 255]} ,
                                 "orange": {"lower": [5, 127, 60], "upper":[22, 255, 255]},
                                 "red": {"lower": [0, 143, 49], "upper":[35, 255, 106]},
                                 "brown": {"lower": [0, 121, 15], "upper":[26, 255, 39]} ,
                                 "green":{"lower": [55, 99, 18], "upper":[76, 255, 237]} ,
                                 "black":{"lower": [31, 21, 0], "upper":[126, 255, 17]} ,
                                 "grey": {"lower": [11, 18, 27], "upper":[56, 108, 96]},
                                }

    def get_boundaries_dict(self):
        return self.color_boundaries
