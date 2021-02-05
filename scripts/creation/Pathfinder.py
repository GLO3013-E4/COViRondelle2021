from PIL import Image, ImageDraw

from params import PATHFINDING_ALGORITHM, NODE_IDENTIFIER_WIDTH


class Pathfinder:
    def __init__(self, map, draw):
        self.map = map
        self.draw = draw
        self.pathfinding_algortihm = PathfindingAlgorithmFactory.create()

    def find_square_matrix_path(self):
        start = self.map.get_start_node()
        end = self.map.get_end_node()
        return self.pathfinding_algortihm.find_path(start, end)

    def show(self):
        self.map.get_image.show()
