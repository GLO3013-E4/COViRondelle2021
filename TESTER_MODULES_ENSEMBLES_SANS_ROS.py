import cv2

from scripts.src.detection.acuro_markers.obstacle_and_robot_finder import ObstacleRobotFinder
from scripts.src.detection.puck_detection import PuckDetection
from scripts.src.pathfinding.show_path import get_path_and_map
from scripts.src.path_following.vectorizer import Vectorizer
from scripts.src.pathfinding.map_drawer import MapDrawer
from scripts.src.pathfinding.pathfinding_algorithms import PathfindingAlgorithms
from scripts.src.pathfinding.config import NODE_SIZE

# TODO: @time_it

colors = {
    "purple",
    "white",
    "yellow",
    "blue",
    "orange",
    "red",
    "brown",
    "green",
    "black",
    "grey"
}


def get_objects(obstacle_and_robot, puck_detection, image):
    obstacles = obstacle_and_robot.detect_obstacle_position(image)
    robot, grip, robot_angle = obstacle_and_robot.detect_robot(image)

    pucks = {
        color: puck_detection.detect_puck(image, color, Debug=False) for color in colors
    }

    return obstacles, pucks, robot, grip, robot_angle


def visualize_vectors(vectorizer, image):
    for i in range(len(vectorizer.corrected_path)-1):
        point1 = vectorizer.corrected_path[i]
        point1 = tuple(map(int, point1))

        point2 = vectorizer.corrected_path[i+1]
        point2 = tuple(map(int, point2))

        thickness = 3

        color = (0, 0, 255)
        if i == 0:
            color = (255, 0, 0)

        image = cv2.arrowedLine(image, point1, point2, color, thickness)
    return image


def visualize_map(_map, path, frame):
    drawer = MapDrawer(15//3, 15, frame)
    drawer.draw_map(_map, path)
    drawer.get_image().show()


def test_on_an_image(image_path: str, goal_color: str):
    obstacle_and_robot_detection = ObstacleRobotFinder()
    puck_detection = PuckDetection()
    vectorizer = Vectorizer(debug=True)

    ret, frame = None, cv2.imread(image_path)
    height, width, channels = frame.shape

    obstacles, pucks, robot_position, grip, robot_angle = get_objects(obstacle_and_robot_detection, puck_detection, frame)

    #TODO: est-ce que c'est ça qu'on veut?
    robot_position = grip

    print(robot_angle)

    # transform obstacles, robot, pucks
    pucks = {key: value for key, value in pucks.items() if value}
    pucks = {
        color: pucks[color]['center_position'] for color in pucks
    }
    obstacles = [
        obstacles[index]['center_of_obstacle'] for index in range(len(obstacles))
    ]

    goal = pucks[goal_color]
    other_pucks = [val for item, val in pucks.items() if item != goal_color]

    path, _map = get_path_and_map(NODE_SIZE, PathfindingAlgorithms.A_STAR, obstacles, robot_position, goal,
                                other_pucks, width, height)

    nodes = [node.pixel_coordinates_center for node in path]
    vectorizer.set_path(nodes)
    vectorizer.set_robot_angle(robot_angle)
    vectorizer.set_robot_position(robot_position)
    vectors = vectorizer.path_to_vectors()

    print(vectors)

    visualize_map(_map, path, frame)
    frame = visualize_vectors(vectorizer, frame)

    cv2.imshow('bleh', frame)
    cv2.waitKey(0)

"""
def test_on_multiple_images_recalculate_path(images: [str], goal_color: str):
    obstacle_detection = ObstacleDetection()
    robot_detection = RobotDetection()
    puck_detection = PuckDetection()
    vectorizer = Vectorizer(debug=True)

    for image_path in images:
        ret, frame = None, cv2.imread(image_path)
        height, width, channels = frame.shape

        obstacles, robot, pucks = get_objects(obstacle_detection, robot_detection, puck_detection, frame)

        # transform obstacles, robot, pucks
        pucks = {key: value for key, value in pucks.items() if value}
        pucks = {
            color: pucks[color]['center_position'] for color in pucks
        }
        obstacles = [
            obstacles[index][f"obstacle {index + 1}"]['center'] for index in range(len(obstacles))
        ]
        robot_position = robot['center']
        robot_angle = get_robot_angle(robot)

        goal = pucks[goal_color]
        other_pucks = [val for item, val in pucks.items() if item != goal_color]

        path, _map = get_path_plus_map(15, "BreadthFirstSearch", obstacles, robot_position, goal, other_pucks, width,
                                       height)

        nodes = [node.pixel_coordinates_center for node in path]
        vectorizer.set_path(nodes)
        vectorizer.set_robot_angle(robot_angle)
        vectorizer.set_robot_position(robot_position)
        vectors = vectorizer.path_to_vectors()

        visualize_map(_map, path, frame)
        frame = visualize_vectors(vectorizer, frame)

        cv2.imshow('bleh', frame)
        cv2.waitKey(0)

    cv2.destroyAllWindows()


def test_on_multiple_images_dont_recalculate_path(images: [str], goal_color: str):
    obstacle_detection = ObstacleDetection()
    robot_detection = RobotDetection()
    puck_detection = PuckDetection()
    vectorizer = Vectorizer(debug=True)

    ret, frame = None, cv2.imread(images[0])
    height, width, channels = frame.shape

    obstacles, robot, pucks = get_objects(obstacle_detection, robot_detection, puck_detection, frame)

    # transform obstacles, robot, pucks
    pucks = {key: value for key, value in pucks.items() if value}
    pucks = {
        color: pucks[color]['center_position'] for color in pucks
    }
    obstacles = [
        obstacles[index][f"obstacle {index + 1}"]['center'] for index in range(len(obstacles))
    ]
    robot_position = robot['center']
    robot_angle = get_robot_angle(robot)

    goal = pucks[goal_color]
    other_pucks = [val for item, val in pucks.items() if item != goal_color]

    path, _map = get_path_plus_map(15, "BreadthFirstSearch", obstacles, robot_position, goal, other_pucks, width,
                                   height)

    nodes = [node.pixel_coordinates_center for node in path]
    vectorizer.set_path(nodes)
    vectorizer.set_robot_angle(robot_angle)
    vectorizer.set_robot_position(robot_position)
    vectors = vectorizer.path_to_vectors()

    visualize_map(_map, path, frame)
    frame = visualize_vectors(vectorizer, frame)
    cv2.imshow('bleh', frame)
    cv2.waitKey(0)

    for image_path in images[1:]:
        ret, frame = None, cv2.imread(image_path)

        obstacles, robot, pucks = get_objects(obstacle_detection, robot_detection, puck_detection, frame)

        robot_position = robot['center']
        robot_angle = get_robot_angle(robot)

        vectorizer.set_robot_angle(robot_angle)
        vectorizer.set_robot_position(robot_position)
        vectors = vectorizer.path_to_vectors()

        # visualize vectors on image
        visualize_map(_map, path, frame)
        frame = visualize_vectors(vectorizer, frame)

        cv2.imshow('bleh', frame)
        cv2.waitKey(0)

    cv2.destroyAllWindows()


def test_on_cam_recalculate_path(goal_color: str):
    cap = cv2.VideoCapture(0)

    obstacle_detection = ObstacleDetection()
    robot_detection = RobotDetection()
    puck_detection = PuckDetection()
    vectorizer = Vectorizer(debug=True)

    while True:
        ret, frame = cap.read()
        height, width, channels = frame.shape

        obstacles, robot, pucks = get_objects(obstacle_detection, robot_detection, puck_detection, frame)

        # transform obstacles, robot, pucks
        pucks = {key: value for key, value in pucks.items() if value}
        pucks = {
            color: pucks[color]['center_position'] for color in pucks
        }
        obstacles = [
            obstacles[index][f"obstacle {index + 1}"]['center'] for index in range(len(obstacles))
        ]
        robot_position = robot['center']
        robot_angle = get_robot_angle(robot)

        goal = pucks[goal_color]
        other_pucks = [val for item, val in pucks.items() if item != goal_color]

        path, _map = get_path_plus_map(15, "BreadthFirstSearch", obstacles, robot_position, goal, other_pucks, width,
                                       height)

        nodes = [node.pixel_coordinates_center for node in path]
        vectorizer.set_path(nodes)
        vectorizer.set_robot_angle(robot_angle)
        vectorizer.set_robot_position(robot_position)
        vectors = vectorizer.path_to_vectors()

        #visualize_map(_map, path, frame)
        frame = visualize_vectors(vectorizer, frame)

        cv2.imshow('bleh', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


def test_on_cam_dont_recalculate_path(goal_color: str):
    cap = cv2.VideoCapture(0)

    obstacle_detection = ObstacleDetection()
    robot_detection = RobotDetection()
    puck_detection = PuckDetection()
    vectorizer = Vectorizer(debug=True)

    ret, frame = cap.read()
    height, width, channels = frame.shape

    obstacles, robot, pucks = get_objects(obstacle_detection, robot_detection, puck_detection, frame)

    # transform obstacles, robot, pucks
    pucks = {key: value for key, value in pucks.items() if value}
    pucks = {
        color: pucks[color]['center_position'] for color in pucks
    }
    obstacles = [
        obstacles[index][f"obstacle {index + 1}"]['center'] for index in range(len(obstacles))
    ]
    robot_position = robot['center']
    robot_angle = get_robot_angle(robot)

    goal = pucks[goal_color]
    other_pucks = [val for item, val in pucks.items() if item != goal_color]

    path, _map = get_path_plus_map(15, "BreadthFirstSearch", obstacles, robot_position, goal, other_pucks, width,
                                   height)

    nodes = [node.pixel_coordinates_center for node in path]
    vectorizer.set_path(nodes)
    vectorizer.set_robot_angle(robot_angle)
    vectorizer.set_robot_position(robot_position)
    vectors = vectorizer.path_to_vectors()

    visualize_map(_map, path, frame)
    frame = visualize_vectors(vectorizer, frame)
    cv2.imshow('bleh', frame)
    cv2.waitKey(0)

    while True:
        ret, frame = cap.read()

        obstacles, robot, pucks = get_objects(obstacle_detection, robot_detection, puck_detection, frame)

        robot_position = robot['center']
        robot_angle = get_robot_angle(robot)

        vectorizer.set_robot_angle(robot_angle)
        vectorizer.set_robot_position(robot_position)
        vectors = vectorizer.path_to_vectors()

        # visualize vectors on image
        #visualize_map(_map, path, frame)
        frame = visualize_vectors(vectorizer, frame)

        cv2.imshow('bleh', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
"""

if __name__ == '__main__':
    GOAL_COLOR = "purple"

    #AN_IMAGE = "./scripts/tests/detection/acuro_marker/robot_5x5_four.jpg"

    AN_IMAGE = "./scripts/src/detection/robot_obstacles4.jpg"
    MULTIPLE_IMAGES = []

    test_on_an_image(AN_IMAGE, GOAL_COLOR)
    # test_on_multiple_images_recalculate_path(MULTIPLE_IMAGES, GOAL_COLOR)
    # test_on_multiple_images_dont_recalculate_path(MULTIPLE_IMAGES, GOAL_COLOR)
    # test_on_cam_dont_recalculate_path(GOAL_COLOR)
    # test_on_cam_recalculate_path(GOAL_COLOR)
