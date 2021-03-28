import math

from scripts.src.path_following.movement_mode import MovementMode


# TODO: devrait savoir le goal + est-ce que le goal est une puck ou une destination. Ou, est-ce que c'est path_plannning.PathFinder qui devrait le savoir ou a_star.

class Vectorizer:
    def __init__(self, minimize=False):
        self.robot_position = None
        self.robot_angle = None
        self.minimize = minimize
        self.path = []
        self.mode = MovementMode.GRIP
        self.cm_to_pixel = 6.8
        self.distance_correction_threshold = 3*self.cm_to_pixel
        self.length_correction_threshold = 0
        self.angle_correction_threshold = 0
        self.goal = None
        self.checkpoint = 0

    def set_mode(self, mode: MovementMode):
        self.mode = mode

    def set_robot_position(self, position: (int, int)):
        self.robot_position = position

    def set_path(self, path: [(float, float)]):
        self.path = path
        #self.checkpoint = 0

    def set_goal(self, goal: (int, int)):
        self.goal = goal

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
                    minimized_vectors[-1] = [last_vector_distance + length, last_vector_angle, last_mode]
        return minimized_vectors

    def get_path_from_robot(self, nodes: [(float, float)]):
        node_distances = [
            distance(self.robot_position, node)
            for node in nodes
        ]
        minimum_distance = min(node_distances)

        if minimum_distance <= self.distance_correction_threshold:
            #if close
            path = [self.robot_position] + nodes[self.checkpoint:] #checkpoint+1?
            self.checkpoint += 1
            return path
        else:
            #if far
            index = node_distances.index(minimum_distance)
            path = [self.robot_position] + nodes[index:]
            self.checkpoint = index + 1
            return path

    def vectorize(self, nodes: [(float, float)]):
        vectors = []
        for i in range(len(nodes)-1):
            x1, y1 = nodes[i]
            x2, y2 = nodes[i+1]
            distance = math.sqrt(pow(x2-x1, 2) + pow(y2-y1, 2))
            angle = -math.atan2(y2-y1, x2-x1)

            if angle == -0:
                angle = 0
            elif angle == -math.pi:
                angle = math.pi

            vector = [distance, angle]
            vectors.append(vector)
        return vectors

    def adjust_vector_angles_from_robot_pov(self, vectors: [[float, float]]):
        """
        Changes the vectors' orientation from their absolute value from the top camera
        to the angle the robot will need to use to align itself with the vector.
        (For all vectors)
        """
        new_vectors = []
        if self.mode is MovementMode.OHMMETER:
            last_vector = [None, self.robot_angle - math.pi/2]
        else:
            last_vector = [None, self.robot_angle]

        for vector in vectors:
            length, angle = self.adjust_vector_angle_from_robot_pov(last_vector, vector)
            if abs(angle) <= self.angle_correction_threshold:
                angle = 0
            if abs(length) <= self.length_correction_threshold:
                length = 0
            new_vectors.append([length, angle, self.mode])
            last_vector = vector
        return new_vectors

    def adjust_vector_angle_from_robot_pov(self, last_vector, current_vector):
        """
        Changes the vector orientation from the absolute value from the top camera
        to the angle the robot will need to use to align itself with the vector.
        (For one vector)
        """
        distance1, angle1 = last_vector
        distance2, angle2 = current_vector
        if angle2 < 0:
            angle2 = 2 * math.pi + angle2
        if angle1 < 0:
            angle1 = 2 * math.pi + angle1

        angle_correction = angle2 - angle1

        if angle_correction > math.pi:
            angle_correction -= 2 * math.pi
        elif angle_correction < -math.pi:
            angle_correction += 2 * math.pi
        return [distance2, angle_correction]

    def set_robot_angle(self, robot_angle):
        self.robot_angle = robot_angle

    def path_to_vectors(self):
        path_from_robot = self.get_path_from_robot(self.path)
        vectors = self.vectorize(path_from_robot + [self.goal])

        vectors[-1][0] = 0

        adjusted_vectors = self.adjust_vector_angles_from_robot_pov(vectors)
        if adjusted_vectors[-1] == [0, 0]:
            adjusted_vectors.pop()

        if self.minimize:
            adjusted_vectors = self.minimize_vectors(adjusted_vectors)

        return adjusted_vectors


def distance(point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    return math.sqrt(pow(x2-x1, 2) + pow(y2-y1, 2))
