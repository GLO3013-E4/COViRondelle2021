class ColorBoundaries:
    """Color boundaries of pucks"""
    def __init__(self):
        self.color_boundaries = {"purple": {"lower": [110, 16, 80], "upper":[172, 245, 255]},
                                 "white": {"lower": [12, 0, 135], "upper":[96, 159, 232]} ,
                                 "yellow": {"lower": [19, 148, 60], "upper":[40, 255, 255]},
                                 "blue": {"lower": [95, 66, 36], "upper":[124, 255, 255]} ,
                                 "orange": {"lower": [6, 139, 77], "upper":[24, 255, 161]},
                                 "red": {"lower": [0, 239, 65], "upper":[7, 255, 143]},
                                 "brown": {"lower": [0, 59, 0], "upper":[41, 255, 57]} ,
                                 "green":{"lower": [56, 74, 18], "upper":[79, 255, 147]} ,
                                 "grey": {"lower": [20, 0, 42], "upper":[96, 157, 111]},
                                 "black":{"lower": [0, 0, 0], "upper":[161, 255, 28]}

                                }

    def get_boundaries_dict(self):
        return self.color_boundaries
