import math

from scripts.src.path_following.config import NODE_SIZE
from scripts.src.path_following.destination import Destination
from scripts.src.path_following.grip_functions import GripFunctions
from scripts.src.path_following.robot_command import RobotCommand


class Vectorizer:
    def __init__(self, minimize=False):
        self.robot_position = None
        self.robot_angle = None
        self.minimize = minimize
        self.path = []
        self.cm_to_pixel = 6.882391855
        self.distance_correction_threshold = 3*self.cm_to_pixel
        self.length_correction_threshold = 0
        self.angle_correction_threshold = 0
        self.checkpoint_trigger_threshold = 3
        self.last_checkpoint = None
        self.goal = None
        self.checkpoint = None
        self.destination = Destination.OTHER

    def set_destination_mode(self, destination: Destination):
        self.destination = destination

    def set_robot_position(self, position: (int, int)):
        self.robot_position = position

        # update checkpoint
        node_distances = [
            i for i, node in enumerate(self.path) if distance(self.robot_position, node) <= NODE_SIZE/2
        ]
        if node_distances:
            if self.checkpoint is None or node_distances[-1] > self.checkpoint:
                self.checkpoint = node_distances[-1]

    def set_path(self, path: [(float, float)]):
        if self.destination is Destination.PUCK or self.destination is Destination.CORNER:
            self.path = self.shorten_path_to_grab_puck(path)
        else:
            self.path = path
        self.checkpoint = None

    def shorten_path_to_grab_puck(self, path: [(float, float)]):
        distances = [distance(node, self.goal) for node in path]
        new_path = []
        for i, distance_from_goal in enumerate(distances):
            new_path.append(path[i])
            if distance_from_goal <= 112:
                break
        return new_path

    def set_goal(self, goal: (int, int)):
        self.goal = goal

    def set_destination(self, destination: Destination):
        self.destination = destination

    def minimize_vectors(self, vectors: [[float, float]]):
        minimized_vectors = []
        for i, vector in enumerate(vectors):
            length, angle, mode = vector
            if angle != 0:
                minimized_vectors.append(vector)
            elif angle == 0:
                if not minimized_vectors:
                    minimized_vectors.append(vector)
                else:
                    last_vector_distance, last_vector_angle, last_mode = minimized_vectors[-1]
                    if mode is last_mode:
                        minimized_vectors[-1] = [last_vector_distance + length, last_vector_angle, last_mode]
        return minimized_vectors

    def get_path_from_robot(self, nodes: [(float, float)]):
        if self.checkpoint is None:
            # va au debut du chemin
            return [self.robot_position] + nodes
            # va au point le plus pret meme si t'as jamais ete dans le chemin par avant
            #node_distances = [
            #    distance(self.robot_position, node)
            #    for node in nodes
            #]
            #minimum_distance = min(node_distances)
            #min_index_no_checkpoint = node_distances.index(minimum_distance)
            # return [self.robot_position] + nodes[min_index_no_checkpoint:]

        node_distances = [
            distance(self.robot_position, node)
            for node in nodes[self.checkpoint+1:]
        ]

        if not node_distances:
            return [self.robot_position]

        minimum_distance = min(node_distances)
        index = node_distances.index(minimum_distance) + self.checkpoint + 1

        if self.robot_is_close_to_path(minimum_distance):
            if self.checkpoint_was_updated_recently():
                return [self.robot_position] + nodes[self.checkpoint+1:]
            else:
                return [self.robot_position] + nodes[index:]
        else:
            return [self.robot_position] + nodes[index:]

    def checkpoint_was_updated_recently(self):
        return distance(self.robot_position, self.path[self.checkpoint]) <= NODE_SIZE

    def robot_is_close_to_path(self, minimum_distance_from_path):
        return minimum_distance_from_path <= self.distance_correction_threshold

    def vectorize(self, nodes: [(float, float)]):
        vectors = []
        for i in range(len(nodes)-1):
            x1, y1 = nodes[i]
            x2, y2 = nodes[i+1]
            length = math.sqrt(pow(x2-x1, 2) + pow(y2-y1, 2))
            angle = -math.atan2(y2-y1, x2-x1)

            if angle == -0:
                angle = 0
            elif angle == -math.pi:
                angle = math.pi

            vector = [length, angle]
            vectors.append(vector)
        return vectors

    def adjust_vector_angles_from_robot_pov(self, vectors: [[float, float]]):
        """
        Changes the vectors' orientation from their absolute value from the top camera
        to the angle the robot will need to use to align itself with the vector.
        (For all vectors)
        """
        new_vectors = []

        robot_angle = self.robot_angle

        for length, angle in vectors:
            # reset towards what angle the directions of the robot are facing
            # calculate their sins and cosines
            # calculate sin and cosine of vector angle
            # check with which direction the difference is smallest
            # set mode to that direction
            # calculate the degree difference between that direction and the vector angle
            # set angle to the vector angle

            sins_and_cosines = {
                mode: (math.sin(mode.value + robot_angle), math.cos(mode.value + robot_angle))
                for mode in RobotCommand
            }

            sin = math.sin(angle)
            cos = math.cos(angle)

            differences = {
                mode: abs(val[0]-sin) + abs(val[1]-cos)
                for mode, val in sins_and_cosines.items()
            }

            min_mode = min(differences, key=differences.get)
            min_mode_angle = min_mode.value + robot_angle

            if min_mode_angle < 0:
                min_mode_angle = 2 * math.pi + min_mode_angle
            if angle < 0:
                angle = 2 * math.pi + angle

            angle_correction = min_mode_angle - angle

            if angle_correction > math.pi:
                angle_correction -= 2 * math.pi
            elif angle_correction < -math.pi:
                angle_correction += 2 * math.pi

            robot_angle = robot_angle + angle_correction

            new_vectors.append([length, angle_correction, min_mode])
        return new_vectors

    def set_robot_angle(self, robot_angle):
        self.robot_angle = robot_angle

    def path_to_vectors(self):
        path_from_robot = self.get_path_from_robot(self.path)

        if self.destination is Destination.PUCK or self.destination is Destination.CORNER:
            vectors = self.vectorize(path_from_robot + [self.goal])

            vectors[-1][0] = 0

            adjusted_vectors = self.adjust_vector_angles_from_robot_pov(vectors)

            length, angle, mode = adjusted_vectors[-1]
            if [length, angle] == [0, 0]:
                adjusted_vectors.pop()

        elif self.destination is Destination.RESISTANCE_STATION:
            vectors = self.vectorize(path_from_robot) + [[0, -math.pi/2]]
            adjusted_vectors = self.adjust_vector_angles_from_robot_pov(vectors)
            length, angle, mode = adjusted_vectors[-1]
            if [length, angle] == [0, 0]:
                adjusted_vectors.pop()
        else:
            vectors = self.vectorize(path_from_robot)
            adjusted_vectors = self.adjust_vector_angles_from_robot_pov(vectors)

        if self.minimize:
            adjusted_vectors = self.minimize_vectors(adjusted_vectors)

        return adjusted_vectors


def distance(point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    return math.sqrt(pow(x2-x1, 2) + pow(y2-y1, 2))
