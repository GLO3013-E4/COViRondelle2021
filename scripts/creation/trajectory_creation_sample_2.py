from scripts.creation.Pathfinder import Pathfinder
from scripts.creation.PathfindingAlgorithmFactory import PathfindingAlgorithmFactory
from scripts.creation.Map import Map
from scripts.creation.MapDrawer import MapDrawer

from PIL import Image


if __name__ == '__main__':
    NODE_SIZE = 25
    NODE_IDENTIFIER_WIDTH = NODE_SIZE / 5
    PATHFINDING_ALGORITHM = "BreadthFirstSearch"

    obstacles = [
        (1142, 290),
        (657, 761),
    ]

    start = (1048, 504)
    end = (244, 370)

    gripper = (1034, 432)

    #TODO: la puck de destination n'est pas dans la liste des pucks
    #TODO: à revoir si c'est ça qu'on veut
    pucks = [
        (241, 288),
        (242, 479),
        (245, 581)
    ]

    image = Image.open("./scripts/data/images/trajectory_example_1.jpg")

    pathfindingAlgorithmFactory = PathfindingAlgorithmFactory()
    pathfinding_algorithm = pathfindingAlgorithmFactory.create(PATHFINDING_ALGORITHM)
    board_map = Map(image, obstacles, pucks, start, end, node_size=NODE_SIZE)
    board_map.render_map()
    map_drawer = MapDrawer(NODE_IDENTIFIER_WIDTH, NODE_SIZE, image)
    pathfinder = Pathfinder(board_map, map_drawer, pathfinding_algorithm)

    pathfinder.find_square_matrix_path()
    pathfinder.show()
