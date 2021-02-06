class Pathfinder:
    def __init__(self, map, map_drawer, pathfinding_algorithm):
        self.map = map
        self.map_drawer = map_drawer
        self.pathfinding_algorithm = pathfinding_algorithm
        self.path = None #TODO: j'aime pas ça tant que ça

    def find_square_matrix_path(self):
        start = self.map.get_start_node()
        end = self.map.get_end_node()
        self.path = self.pathfinding_algorithm.find_path(start, end)

    def show(self):
        self.map_drawer.draw_map(self.map, self.path)
        self.map_drawer.get_image().show()
