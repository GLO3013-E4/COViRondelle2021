from Pathfinder import Pathfinder
from PathfindingAlgorithmFactory import PathfindingAlgorithmFactory
from params import NODE_SIZE, SAFETY_CUSHION, ROBOT_WIDTH, OBSTACLE_WIDTH, PUCK_WIDTH, OBSTACLE_CUSHION_WIDTH, NODE_IDENTIFIER_WIDTH, PATHFINDING_ALGORITHM
from Map import Map
from MapDrawer import MapDrawer

from PIL import Image


if __name__ == '__main__':
    obstacles = [
        (175, 27),
        (175, 75),
        (175, 123),
        (175, 170),
        (175, 222),
        (225, 228)
    ]

    start = (21, 24)
    end = (27, 382)

    pucks = [
        #(175, 27),
        #(175, 75),
        #(175, 123),
        #(175, 170),
        #(175, 222),
        #(225, 228)
    ]

    #image = Image.open("../../../../Desktop/photo_camera_monde/WIN_20210126_11_29_53_Pro.jpg")
    image = Image.open("../../../../Desktop/Q5jkZwR.png")

    pathfinding_algorithm = PathfindingAlgorithmFactory.create(PATHFINDING_ALGORITHM)
    board_map = Map(image, NODE_SIZE, SAFETY_CUSHION, ROBOT_WIDTH, OBSTACLE_WIDTH, PUCK_WIDTH, OBSTACLE_CUSHION_WIDTH, obstacles, pucks, start, end)
    map_drawer = MapDrawer(NODE_IDENTIFIER_WIDTH, NODE_SIZE, image)
    pathfinder = Pathfinder(board_map, map_drawer, pathfinding_algorithm)

    pathfinder.find_square_matrix_path()
    pathfinder.show()
