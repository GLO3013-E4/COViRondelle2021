from PIL import Image

from scripts.creation.pathfinder import Pathfinder
from scripts.creation.pathfinding_algorithm_factory import PathfindingAlgorithmFactory
from scripts.creation.map import Map
from scripts.creation.map_drawer import MapDrawer


if __name__ == '__main__':
    NODE_SIZE = 25
    NODE_IDENTIFIER_WIDTH = NODE_SIZE / 5
    PATHFINDING_ALGORITHM = "BreadthFirstSearch"

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

    IMAGE = Image.open("./scripts/data/images/trajectory_example_1.jpg")

    pathfindingAlgorithmFactory = PathfindingAlgorithmFactory()
    pathfinding_algorithm = pathfindingAlgorithmFactory.create(PATHFINDING_ALGORITHM)
    board_map = Map(IMAGE, OBSTACLES, PUCKS, START, END, node_size=NODE_SIZE)
    board_map.render_map()
    map_drawer = MapDrawer(NODE_IDENTIFIER_WIDTH, NODE_SIZE, IMAGE)
    pathfinder = Pathfinder(board_map, map_drawer, pathfinding_algorithm)

    pathfinder.find_square_matrix_path()
    pathfinder.show()

