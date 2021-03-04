from enum import Enum


class QrCodeTypes(Enum):
    ROBOT = "robot"
    OBSTACLE = "obstacle"
    ROBOT_AND_OBSTACLE = "both"