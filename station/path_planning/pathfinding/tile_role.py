"""Enum that specifies the different roles the nodes can have in the graph"""

from enum import Enum


class TileRole(Enum):
    """Enum that specifies the different roles the nodes can have in the graph"""
    START = 0
    END = 1
    OBSTACLE = 2
    CUSHION = 3
    PUCK = 4
    EMPTY = 5
