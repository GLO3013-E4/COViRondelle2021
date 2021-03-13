import math

# TODO: SUPER IMPORTANT QUE CE SOIT LE MÊME QUE LUI DANS LE NOEUD DE PATHFINDING.
# TODO: QU'EST-CE QU'ON DEVRAIT FAIRE POUR S'ASSURER QUE C'EST LE MÊME
NODE_SIZE = 15


class Vectorizer:
    def __init__(self):
        self.robot_position = None
        self.robot_angle = None

    def set_robot_position(self, position):
        self.robot_position = position

    @staticmethod
    def smooth_path(path: [(int, int)]):
        # TODO: c'est deg
        smoothed_path = []

        i = 0
        while i < len(path)-2:
            x, y = path[i]
            diagonals = [
                (x - NODE_SIZE, y - NODE_SIZE),
                (x - NODE_SIZE, y + NODE_SIZE),
                (x + NODE_SIZE, y - NODE_SIZE),
                (x + NODE_SIZE, y + NODE_SIZE)
            ]
            if path[i+2] in diagonals:
                smoothed_path.append(path[i])
                i += 2
            else:
                smoothed_path.append(path[i])
                i += 1

        for j in range(i, len(path)):
            smoothed_path.append(path[j])
        return smoothed_path

    def minimize_vectors(self, vectors: [(float, float)]):
        minimized_vectors = []
        for i, vector in enumerate(vectors):
            distance, angle = vector
            if angle != 0:
                minimized_vectors.append(vector)
            elif angle == 0:
                if not minimized_vectors:
                    minimized_vectors.append(vector)
                else:
                    last_vector_distance, last_vector_angle = minimized_vectors[-1]
                    minimized_vectors[-1] = (last_vector_distance + distance, last_vector_angle)
        return minimized_vectors

    def correct_path(self, nodes: [(int, int)]):
        #TODO:
        # jveux tu (distance_from _robot - distance_from_goal) parce que peut-être qu'il va
        # souvent essayer de retourner en arrière. Si nos carrés sont trop petits la fonction
        # de correct path va être trop sensible aussi. À la place on pourrait calculer
        # la distance et ensuite si elle est plus grande qu'un certain threshold on
        # applique une correction?

        robot_node = (
            (self.robot_position[0]//NODE_SIZE)*NODE_SIZE,
            (self.robot_position[1]//NODE_SIZE)*NODE_SIZE
        )
        if robot_node in nodes:
            index = nodes.index(robot_node)
            return nodes[index:]

        elif robot_node not in nodes:
            x, y = self.robot_position

            distance_from_robot = [
                math.sqrt(pow(x2-x, 2) + pow(y2-y, 2)) for (x2, y2) in nodes
            ]
            minimum_distance = min(distance_from_robot)
            index = distance_from_robot.index(minimum_distance)
            corrected_path = [self.robot_position] + nodes[index:]
            return corrected_path

    def vectorize(self, nodes: [(int, int)]):
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

            vector = (distance, angle)
            vectors.append(vector)
        return vectors

    def adjust_vector_angles_from_robot_pov(self, vectors: [(float, float)]):
        """
        Changes the vectors' orientation from their absolute value from the top camera
        to the angle the robot will need to use to align itself with the vector.
        (For all vectors)
        """
        new_vectors = []
        last_vector = (None, self.robot_angle)
        for vector in vectors:
            new_vectors.append(self.adjust_vector_angle_from_robot_pov(last_vector, vector))
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
        return distance2, angle_correction

    def set_robot_angle(self, robot_angle):
        self.robot_angle = robot_angle

    def path_to_vectors(self, nodes: [(int, int)], to_min=False):
        smoothed_path = self.smooth_path(nodes)
        corrected_path = self.correct_path(smoothed_path)
        vectors = self.vectorize(corrected_path)
        adjusted_vectors = self.adjust_vector_angles_from_robot_pov(vectors)

        if to_min:
            adjusted_vectors = self.minimize_vectors(adjusted_vectors)

        return adjusted_vectors
