from PIL import Image

from scripts.src.pathfinding.pathfinder import Pathfinder
from scripts.src.pathfinding.pathfinding_algorithm_factory import PathfindingAlgorithmFactory
from scripts.src.pathfinding.map import Map
from scripts.src.pathfinding.map_drawer import MapDrawer


def show_path(node_size, algorithm, obstacles, start, end, pucks, image_path):
    """Example of how to find a path between the robot and a specific puck
    given the obstacles, a start position, an end position and where the other pucks are."""
    node_identifier_width = node_size / 5
    image = Image.open(image_path)

    pathfinding_algorithm_factory = PathfindingAlgorithmFactory()
    pathfinding_algorithm = pathfinding_algorithm_factory.create(algorithm)
    board_map = Map(image, obstacles, pucks, start, end, node_size=node_size)
    board_map.render_map()
    map_drawer = MapDrawer(node_identifier_width, node_size, image)
    pathfinder = Pathfinder(board_map, map_drawer, pathfinding_algorithm)

    pathfinder.find_square_matrix_path()
    pathfinder.show()
