"""Example of how to find a path between the robot and a specific puck
given the obstacles, a start position, an end position and where the other pucks are."""

from PIL import Image

from scripts.creation.pathfinder import Pathfinder
from scripts.creation.pathfinding_algorithm_factory import PathfindingAlgorithmFactory
from scripts.creation.map import Map
from scripts.creation.map_drawer import MapDrawer


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


if __name__ == '__main__':
    NODE_SIZE = 25
    NODE_IDENTIFIER_WIDTH = NODE_SIZE / 5
    ALGORITHM = "BreadthFirstSearch"

    OBSTACLES = [
        (1142, 290),
        (657, 761),
    ]

    START = (1048, 504)
    END = (526, 418)

    PUCKS = [
        (241, 288),
        (373, 277),
        (332, 515),
        (250, 588),
        (807, 341),
        (800, 435),
        (745, 787),
        (1093, 655),
        (1150, 215)
    ]

    IMAGE_PATH = "./scripts/data/images/trajectory_example_1.jpg"
    show_path(NODE_SIZE, ALGORITHM, OBSTACLES, START, END, PUCKS, IMAGE_PATH)
