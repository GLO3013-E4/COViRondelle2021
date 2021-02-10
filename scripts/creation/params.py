PATHFINDING_ALGORITHM = "BFS"


# Quand on va vouloir bouger en diagonale et qu'on voudra que les coussins soient plus sphériques
#OBSTACLE_CUSHION_45 = SAFETY_CUSHION + (OBSTACLE_WIDTH + (ROBOT_WIDTH/2))/(math.cos(45)*180/math.pi)  # instead of dividing it by node_size, divide it by root(2)*node_size
#PUCK_CUSHION_45 = SAFETY_CUSHION + (PUCK_WIDTH + (ROBOT_WIDTH/2))/(math.cos(45)*180/math.pi)  # instead of dividing it by node_size, divide it by root(2)*node_size


# only for drawing purposes.
NODE_IDENTIFIER_WIDTH = 25 / 5  # NODE_SIZE / 5
