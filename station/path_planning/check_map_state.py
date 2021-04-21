import cv2
import numpy as np

from pathfinding.pathfinder import Pathfinder
from pathfinding.pathfinding_algorithm_factory import PathfindingAlgorithmFactory
from pathfinding.map import Map
from pathfinding.map_drawer import MapDrawer
from pathfinding.pathfinding_algorithms import PathfindingAlgorithms
from pathfinding.path_not_found_exception import PathNotFoundException
from pathfinding.tile_role import TileRole
from pathfinding.config import NODE_SIZE, SAFETY_CUSHION, OBSTACLE_WIDTH, PUCK_WIDTH

import sys
sys.path.append("../robot_goal_and_obstacle_finder")
from detection.acuro_markers.obstacle_and_robot_finder import \
    ObstacleRobotFinder
from detection.puck_detection import PuckDetection


node_identifier_width = NODE_SIZE // 5
robot_and_obstacle_finder = ObstacleRobotFinder()
puck_finder = PuckDetection()


def get_map_and_pathfinder(algorithm, obstacles, start, end, pucks, image_width, image_height, safety_cushion, puck_width, obstacle_width, puck_goal_coordinates):
    pathfinding_algorithm_factory = PathfindingAlgorithmFactory()
    pathfinding_algorithm = pathfinding_algorithm_factory.create(algorithm)
    board_map = Map(image_width, image_height, obstacles, pucks, start, end,
                    node_size=NODE_SIZE,
                    safety_cushion=safety_cushion, puck_width=puck_width,
                    obstacle_width=obstacle_width, puck_goal_coordinates=puck_goal_coordinates)
    board_map.render_map()
    pathfinder = Pathfinder(board_map, pathfinding_algorithm)
    return pathfinder, board_map


def check_map_state_on_image(image, color, position):
    height, width, channels = image.shape
    pucks_dict = puck_finder.detect_pucks(image)
    #print(pucks_dict)
    obstacles_list = robot_and_obstacle_finder.detect_obstacle_position(image, DEBUG=False)
    center_of_bottom_robot, prehenseur_position, angle_robot = robot_and_obstacle_finder.detect_robot(image)

    pucks = [single["center_position"] for color in pucks_dict for single in pucks_dict[color]]
    obstacles = [single["center_of_obstacle"] for single in obstacles_list]

    start = center_of_bottom_robot

    if color:
        end = pucks_dict[color][0]["center_position"]
        puck_goal_coordinates = end
    elif position:
        end = position
        puck_goal_coordinates = False
    else:
        end = (500, 500)
        puck_goal_coordinates = False

    pucks = [puck for puck in pucks if puck != end]

    safety_cushion = SAFETY_CUSHION
    puck_width = PUCK_WIDTH
    obstacle_width = OBSTACLE_WIDTH
    found_path = False

    while not found_path:
        pathfinder, board_map = get_map_and_pathfinder(PathfindingAlgorithms.BREADTH_FIRST_SEARCH, obstacles,
                                                       start, end, pucks, width, height, safety_cushion, puck_width,
                                                       obstacle_width, puck_goal_coordinates)
        try:
            pathfinder.find_square_matrix_path()
            path = pathfinder.path
            found_path = True
        except PathNotFoundException:
            print("path not found")
            if board_map.get_node_from_pixel(start).role is TileRole.OBSTACLE:
                print("the robot is inside an obstacle")
                obstacle_width = max(0, obstacle_width - NODE_SIZE)
            elif board_map.get_node_from_pixel(end).role is TileRole.OBSTACLE:
                print("the goal is inside an obstacle")
                obstacle_width = max(0, obstacle_width - NODE_SIZE())
            elif board_map.get_node_from_pixel(end).role is TileRole.PUCK:
                print("the goal is inside a puck")
                puck_width = max(0, puck_width - NODE_SIZE)
            else:
                print("there's really no path between the two", board_map.get_node_from_pixel(end).role, board_map.get_node_from_pixel(start).role)
            path = []

    map_drawer = MapDrawer(node_identifier_width, NODE_SIZE, image)
    map_drawer.draw_map(board_map, path)
    image = map_drawer.get_image()

    open_cv_image = np.array(image.convert('RGB'))
    open_cv_image = open_cv_image[:, :, ::-1].copy()
    return open_cv_image


def check_map_state_on_a_stream(color, position):
    cap = cv2.VideoCapture(2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1600)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 904)

    while True:
        ret, image = cap.read()
        while image is None:
            ret, image = cap.read()

        image = check_map_state_on_image(image, color, position)

        cv2.imshow('', image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    #color = "red"
    color = "orange"
    position = None
    #color = None
    #position = (400, 400)

    image_path = "WIN_20210302_12_53_39_Pro.jpg"
    image = cv2.imread(image_path)
    image = check_map_state_on_image(image, color, position)
    cv2.imshow('', image)
    cv2.waitKey(0)


    #check_map_state_on_a_stream(color, position)

