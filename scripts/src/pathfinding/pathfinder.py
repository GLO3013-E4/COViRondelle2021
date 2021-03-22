"""Module used to find and draw a path from a point to another using a map representation"""
from scripts.src.util.time_it import time_it


class Pathfinder:
    """Class used to find and draw a path from a point to another using a map representation"""
    def __init__(self, _map, pathfinding_algorithm):
        self._map = _map
        self.pathfinding_algorithm = pathfinding_algorithm
        self.path = []

    @time_it
    def find_square_matrix_path(self):
        """Finds the path from the starting node to the end node"""
        start = self._map.get_start_node()
        end = self._map.get_end_node()
        self.path = self.pathfinding_algorithm.find_path(start, end)
