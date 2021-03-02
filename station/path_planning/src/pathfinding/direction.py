"""Enum that specifies the directions the robot can move in its graph"""

from enum import Enum


class Direction(Enum):
    """Enum that specifies the directions the robot can move in its graph"""
    RIGHT = 0
    TOP_RIGHT = 1
    UP = 2
    TOP_LEFT = 3
    LEFT = 4
    DOWN_LEFT = 5
    DOWN = 6
    DOWN_RIGHT = 7
