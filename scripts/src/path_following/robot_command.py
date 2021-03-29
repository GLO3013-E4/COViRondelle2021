import math

from enum import Enum


class RobotCommand(Enum):
    FORWARD = 0
    FORWARD_RIGHT = -math.pi/4
    RIGHT = -math.pi/2
    BACKWARDS_RIGHT = -3*math.pi/4
    BACKWARDS = math.pi
    BACKWARDS_LEFT = 3*math.pi/4
    LEFT = math.pi/2
    FORWARD_LEFT = math.pi/4

    def get_mode_mapping(self):
        return {
            self.FORWARD: 0,
            self.FORWARD_RIGHT: 0,
            self.RIGHT: 0,
            self.BACKWARDS_RIGHT: 0,
            self.BACKWARDS: 0,
            self.BACKWARDS_LEFT: 0,
            self.LEFT: 0,
            self.FORWARD_LEFT: 0
        }
