from scripts.src.pathfinding.node import Node
#smooth escaliers

#vectorize min

#vectorize max

#correct path

#TODO: se rappeller que vectorizer va tout le temps avoir le full path.
# Donc, recalculer les vecteurs à envoyer selon le robot est où dans le path ou de où off il est.

#TODO: SUPER IMPORTANT QUE CE SOIT LE MÊME QUE LUI DANS LE NOEUD DE PATHFINDING.
#TODO: QU'EST-CE QU'ON DEVRAIT FAIRE POUR S'ASSURER QUE C'EST LE MÊME
NODE_SIZE = 15

class Vectorizer:
    def __init__(self):
        self.robot_position = None

    def set_robot_position(self, position):
        self.robot_position = position

    def smooth_path(self, path: [Node]):
        #TODO: c'est deg
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
                if i > len(path)-2:
                    smoothed_path.append(path[i])
                continue
            else:
                smoothed_path.append(path[i])
                i += 1
        return smoothed_path

    def minimize_nodes(self, nodes: [(int, int)]):
        pass

    def maximize_nodes(self, nodes: [(int, int)]):
        pass

    def correct_path(self, nodes: [(int, int)]):
        # jveux tu distance from robot - distance from goal parce que peut-être qu'il va
        # souvent essayer de retourner en arrière. Si nos carrés sont trop petit la fonction de correct
        # path va être trop sensible aussi. À la place on pourrait calculer la distance et ensuite si
        # elle est plus grand qu'un certain nombre on fait une correction?

        robot_node = (self.robot_position[0]//NODE_SIZE, self.robot_position[1]//NODE_SIZE)
        if robot_node in nodes:
            index = nodes.index(robot_node)
            return nodes[index:]

        elif robot_node not in nodes:
            x, y = self.robot_position

            distance_from_robot = [ #jveux tu distance from robot - distance from goal parce que peut-être qu'il va souvent essayer de retourner en arrière. Si nos carrés sont trop petit la fonction de correct path va être trop sensible aussi. À la place on pourrait calculer la distance et ensuite si elle est plus grand qu'un certain nombre on fait une correction?
                ()
            ]


