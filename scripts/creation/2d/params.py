NODE_SIZE = 25  # pixels
SAFETY_CUSHION = 0  # pixels
ROBOT_WIDTH = 200/2  # pixels
OBSTACLE_HEIGHT = OBSTACLE_WIDTH = 40  # pixels
ROBOT_HEIGHT = 170/2  # pixels
PUCK_HEIGHT = PUCK_WIDTH = 25  # pixels

OBSTACLE_CUSHION_WIDTH = SAFETY_CUSHION + ROBOT_WIDTH + OBSTACLE_WIDTH
OBSTACLE_CUSHION_HEIGHT = SAFETY_CUSHION + ROBOT_HEIGHT + OBSTACLE_HEIGHT

PUCK_CUSHION_WIDTH = SAFETY_CUSHION + ROBOT_WIDTH + PUCK_WIDTH
PUCK_CUSHION_HEIGHT = SAFETY_CUSHION + ROBOT_HEIGHT + PUCK_HEIGHT

PATHFINDING_ALGORITHM = "BFS"


# Quand on va vouloir bouger en diagonale et qu'on voudra que les coussins soient plus sphériques
#OBSTACLE_CUSHION_45 = SAFETY_CUSHION + (OBSTACLE_WIDTH + (ROBOT_WIDTH/2))/(math.cos(45)*180/math.pi)  # instead of dividing it by node_size, divide it by root(2)*node_size
#PUCK_CUSHION_45 = SAFETY_CUSHION + (PUCK_WIDTH + (ROBOT_WIDTH/2))/(math.cos(45)*180/math.pi)  # instead of dividing it by node_size, divide it by root(2)*node_size


# only for drawing purposes.
NODE_IDENTIFIER_WIDTH = NODE_SIZE / 5
