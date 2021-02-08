from Pathfinder import Pathfinder
from PathfindingAlgorithmFactory import PathfindingAlgorithmFactory
from params import NODE_SIZE, SAFETY_CUSHION, ROBOT_WIDTH, OBSTACLE_WIDTH, PUCK_WIDTH, OBSTACLE_CUSHION_WIDTH, NODE_IDENTIFIER_WIDTH, PATHFINDING_ALGORITHM, PUCK_CUSHION_WIDTH
from Map import Map
from MapDrawer import MapDrawer

from PIL import Image


if __name__ == '__main__':
    obstacles = [
        (1142, 290),
        (657, 761),
    ]

    start = (1048, 504)
    end = (526, 418)

    #TODO: la puck de destination n'est pas dans la liste des pucks
    #TODO: à revoir si c'est ça qu'on veut
    pucks = [
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

    image = Image.open("./WIN_20210126_11_29_53_Pro.jpg")

    pathfindingAlgorithmFactory = PathfindingAlgorithmFactory()
    pathfinding_algorithm = pathfindingAlgorithmFactory.create(PATHFINDING_ALGORITHM)
    board_map = Map(image, NODE_SIZE, SAFETY_CUSHION, ROBOT_WIDTH, OBSTACLE_WIDTH, PUCK_WIDTH, OBSTACLE_CUSHION_WIDTH, obstacles, pucks, PUCK_CUSHION_WIDTH, start, end)
    map_drawer = MapDrawer(NODE_IDENTIFIER_WIDTH, NODE_SIZE, image)
    pathfinder = Pathfinder(board_map, map_drawer, pathfinding_algorithm)

    pathfinder.find_square_matrix_path()
    pathfinder.show()

