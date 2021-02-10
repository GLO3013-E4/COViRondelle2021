from enum import Enum, auto


class TileRole(Enum):
    START = auto(),
    END = auto(),
    OBSTACLE = auto(),
    CUSHION = auto(),
    PUCK = auto(),
    EMPTY = auto()
