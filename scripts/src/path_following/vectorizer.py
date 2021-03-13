import math

from scripts.src.pathfinding.node import Node

#TODO: se rappeller que vectorizer va tout le temps avoir le full path.
# Donc, recalculer les vecteurs à envoyer selon le robot est où dans le path ou de où off il est.

# TODO: SUPER IMPORTANT QUE CE SOIT LE MÊME QUE LUI DANS LE NOEUD DE PATHFINDING.
# TODO: QU'EST-CE QU'ON DEVRAIT FAIRE POUR S'ASSURER QUE C'EST LE MÊME
NODE_SIZE = 15


class Vectorizer:
    def __init__(self):
        self.robot_position = None

    def set_robot_position(self, position):
        self.robot_position = position

    @staticmethod
    def smooth_path(path: [Node]):
        # TODO: c'est deg
        smoothed_path = []

        i = 0
        while i < len(path)-2:
            x, y = path[i].matrix_center
            diagonals = [
                (x - 1, y - 1),
                (x - 1, y + 1),
                (x + 1, y - 1),
                (x + 1, y + 1)
            ]
            if path[i+2].matrix_center in diagonals:
                smoothed_path.append(path[i])
                i += 2
                continue
            else:
                smoothed_path.append(path[i])
                i += 1

        for j in range(i, len(path)):
            smoothed_path.append(path[j])
        return smoothed_path

    # TODO:
    def minimize_nodes(self, nodes: [(int, int)]):
        pass

    # TODO:
    def maximize_nodes(self, nodes: [(int, int)]):
        pass

    # TODO: add tests correct_path
    def correct_path(self, nodes: [(int, int)]):
        #TODO:
        # jveux tu (distance_from _robot - distance_from_goal) parce que peut-être qu'il va
        # souvent essayer de retourner en arrière. Si nos carrés sont trop petits la fonction
        # de correct path va être trop sensible aussi. À la place on pourrait calculer
        # la distance et ensuite si elle est plus grande qu'un certain threshold on
        # applique une correction?

        robot_node = (self.robot_position[0]//NODE_SIZE, self.robot_position[1]//NODE_SIZE)
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

            return [self.robot_position] + nodes[index:]

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

    # TODO:
    def adjust_angle_from_robot_pov(self, vectors: [(float, float)]):
        pass
