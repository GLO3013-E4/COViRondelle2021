"""
Class that represents where the robot can move and where the
different obstacles and objects laying on the table are.
"""

from scripts.src.pathfinding.node import Node
from scripts.src.pathfinding.tile_role import TileRole
from scripts.src.pathfinding.direction import Direction


class Map:
    """
    Class that represents where the robot can move and where the
    different obstacles and objects laying on the table are.
    """
    def __init__(self, image, obstacles, pucks, start, end, node_size=25, safety_cushion=0,
                 robot_width=100, obstacle_width=40, puck_width=25):
        self.node_size = node_size
        self.safety_cushion = safety_cushion
        self.robot_width = robot_width
        self.obstacle_width = obstacle_width
        self.puck_width = puck_width
        self.obstacle_cushion_width = self.safety_cushion + self.robot_width + self.obstacle_width
        self.obstacle_puck_width = self.safety_cushion + self.robot_width + self.puck_width

        self.image = image
        self.width, self.height = self.image.size

        self.obstacles = obstacles
        self.pucks = pucks
        self.start_node_location = start
        self.end_node_location = end

        self.node_matrix = []

    def render_map(self):
        """Creates the nodes and generates the obstacles, pucks, start and end node."""
        self.create_nodes()
        self.connect_nodes()
        self.create_obstacles()
        self.create_pucks()
        self.create_start_node()
        self.create_end_node()

    def create_nodes(self):
        """Creates the matrix containing the nodes."""
        node_matrix = [
            [] for _ in range((self.height // self.node_size) + 1)
        ]

        for column in range((self.height // self.node_size)+1):
            for row in range((self.width // self.node_size)+1):
                y_position = column * self.node_size + self.node_size / 2
                x_position = row * self.node_size + self.node_size / 2
                node_matrix[column].append(
                    Node((column, row), (x_position, y_position), self.node_size, self.node_size))

        self.node_matrix = node_matrix

    def connect_nodes(self):
        """Connect each node to its neighbors. This method basically
        defines what movements are allowed by the robot."""
        for line in self.node_matrix:
            for node in line:
                possible_neighbors = [
                    (y_position, x_position, direction)

                    for (y_position, x_position, direction) in
                    [
                        (
                            node.matrix_center[0] - 1,
                            node.matrix_center[1],
                            Direction.UP
                        ),
                        (
                            node.matrix_center[0] + 1,
                            node.matrix_center[1],
                            Direction.DOWN
                        ),
                        (
                            node.matrix_center[0],
                            node.matrix_center[1] - 1,
                            Direction.LEFT
                        ),
                        (
                            node.matrix_center[0],
                            node.matrix_center[1] + 1,
                            Direction.RIGHT
                        ),

                        # (node.center[0] - 1, node.matrix_center[1] - 1, Direction.TOP_LEFT),
                        # (node.center[0] - 1, node.matrix_center[1] + 1, Direction.TOP_RIGHT),
                        # (node.center[0] + 1, node.matrix_center[1] - 1, Direction.DOWN_LEFT),
                        # (node.center[0] + 1, node.matrix_center[1] + 1, Direction.DOWN_RIGHT),
                    ]

                    if ((0 <= x_position < len(self.node_matrix[0])
                         and 0 <= y_position < len(self.node_matrix))
                        and (x_position, y_position) != node.matrix_center)
                ]

                for (y_position, x_position, direction) in possible_neighbors:
                    node.neighbors.append((self.node_matrix[y_position][x_position], direction))

    def add_cushion(self, node, distance, role):
        """This method is used to add padding to the obstacles"""
        if distance > 0:
            for neighbor, _ in node.neighbors:
                if neighbor.role is TileRole.EMPTY:
                    neighbor.role = role
                self.add_cushion(neighbor, distance - 1, role)

    def add_cushion_in_direction(self, node, distance, role, direction):
        if distance > 0:
            for neighbor, neighbor_direction in node.neighbors:
                if neighbor.role is TileRole.EMPTY and neighbor_direction is direction:
                    neighbor.role = role
                    self.add_cushion_in_direction(neighbor, distance - 1, role, direction)

    def create_obstacles(self):
        """Specifies which nodes should be considered as obstacles and then adds their padding."""
        for pixel_position in self.obstacles:
            node = self.get_node_from_pixel(pixel_position)
            node.role = TileRole.OBSTACLE

            distance = (self.obstacle_cushion_width // self.node_size) + 1
            self.add_cushion(node, distance, TileRole.CUSHION)

    def create_pucks(self):
        """Specifies which nodes should be considered as pucks and then adds their padding."""
        for pixel_position in self.pucks:
            node = self.get_node_from_pixel(pixel_position)
            node.role = TileRole.PUCK

            distance = (self.obstacle_puck_width // self.node_size) + 1
            self.add_cushion(node, distance, TileRole.CUSHION)

    def create_start_node(self):
        """Specifies which node should be considered as the starting node."""
        start = self.get_start_node()
        start.role = TileRole.START

    def create_end_node(self):
        """Specifies which node should be considered as the end node and then
        adds padding to it. The padding part is there because in some configurations
        where the pucks were close to each other, sometimes their cushions were overlapping
        which was putting the end node out of reach when in reality it should be able to be
        picked up by the robot. Thus, the end node has the same padding as the obstacles,
        which should put the robot next to the end node at the end of its path
        (instead of blindly running into the puck to get to where we identified
        the center of the puck was)."""
        end = self.get_end_node()
        end.role = TileRole.END

        distance = (self.obstacle_puck_width // self.node_size) + 1
        # TODO:
        #self.add_cushion_in_direction(end, distance, TileRole.END, Direction.DOWN)
        #self.add_cushion_in_direction(end, distance, TileRole.END, Direction.LEFT)
        #self.add_cushion_in_direction(end, distance, TileRole.END, Direction.UP)
        #self.add_cushion_in_direction(end, distance, TileRole.END, Direction.RIGHT)

    def get_start_node(self):
        """Gets the starting node"""
        return self.get_node_from_pixel(self.start_node_location)

    def get_end_node(self):
        """Gets the end node"""
        return self.get_node_from_pixel(self.end_node_location)

    def get_image(self):
        """Gets the image"""
        return self.image

    def get_node_matrix(self):
        """Gets the node matrix"""
        return self.node_matrix

    def get_node_from_pixel(self, pixel):
        """Gets a node using any of the pixels the node should cover"""
        x_position = pixel[0] // self.node_size
        y_position = pixel[1] // self.node_size
        return self.node_matrix[y_position][x_position]

    def get_node_from_matrix_coordinates(self, coordinates):
        """Gets a node using its position in the node matrix"""
        x_position, y_position = coordinates
        return self.node_matrix[y_position][x_position]
