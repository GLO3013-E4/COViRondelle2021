NODE_SIZE = 50  # pixels
SAFETY_CUSHION = 0  # pixels
ROBOT_WIDTH = 30  # pixels
OBSTACLE_WIDTH = 30  # pixels
PUCK_WIDTH = 30  # pixels
OBSTACLE_CUSHION_WIDTH = SAFETY_CUSHION + ROBOT_WIDTH
PATHFINDING_ALGORITHM = "BFS"

# amount of space of a square(node) it will take to identify it as part of the path / an obstacle / etc.
# only for drawing purposes.
NODE_IDENTIFIER_WIDTH = NODE_SIZE / 5
