"""Module used to find and draw a path from a point to another using a map representation"""


class Pathfinder:
    """Class used to find and draw a path from a point to another using a map representation"""
    def __init__(self, _map, map_drawer, pathfinding_algorithm):
        self._map = _map
        self.map_drawer = map_drawer
        self.pathfinding_algorithm = pathfinding_algorithm
        self.path = []

    def find_square_matrix_path(self):
        """Finds the path from the starting node to the end node"""
        start = self._map.get_start_node()
        end = self._map.get_end_node()
        self.path = self.pathfinding_algorithm.find_path(start, end)

    def show(self):
        """Draws the map and the path used to get from the starting node to the end node"""
        self.map_drawer.draw_map(self._map, self.path)
        self.map_drawer.get_image().show()
