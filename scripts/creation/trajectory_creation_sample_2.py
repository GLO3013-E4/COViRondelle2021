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
    END = (244, 370)

    GRIPPER = (1034, 432)

    PUCKS = [
        (241, 288),
        (242, 479),
        (245, 581)
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
