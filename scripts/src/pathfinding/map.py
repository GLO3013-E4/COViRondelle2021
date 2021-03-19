"""
Class that represents where the robot can move and where the
different obstacles and objects laying on the table are.
"""
import math

from scripts.src.pathfinding.node import Node
from scripts.src.pathfinding.tile_role import TileRole
from scripts.src.pathfinding.direction import Direction
from scripts.src.pathfinding.obstacle_representation import ObstacleRepresentation


class Map:
    """
    Class that represents where the robot can move and where the
    different obstacles and objects laying on the table are.
    """
    def __init__(self, image_width, image_height, obstacles, pucks, start, end, node_size=25,
                 safety_cushion=0, robot_width=100, obstacle_width=40, puck_width=25,
                 obstacle_representation=ObstacleRepresentation.RADIUS):
        self.node_size = node_size
        self.safety_cushion = safety_cushion
        self.robot_width = robot_width
        self.obstacle_width = obstacle_width
        self.puck_width = puck_width
        self.obstacle_cushion_width = self.safety_cushion + self.robot_width + self.obstacle_width
        self.obstacle_puck_width = self.safety_cushion + self.robot_width + self.puck_width
        self.obstacle_representation = obstacle_representation

        self.width, self.height = image_width, image_height

        self.obstacles = obstacles
        self.pucks = pucks
        self.start_node_location = start
        self.end_node_location = end

        self.node_matrix = []

        self.table_walls_start_y = 40
        self.table_walls_end_y = 810
        self.table_walls_start_x = 0
        self.table_walls_end_x = 1600

    def render_map(self):
        """Creates the nodes and generates the obstacles, pucks, start and end node."""
        self.create_nodes()
        self.connect_nodes()
        self.add_table_walls()
        self.create_obstacles()
        self.create_pucks()
        self.create_start_node()
        self.create_end_node()

    def add_top_wall(self, width):
        start_wall_top = max(0, self.table_walls_start_y // self.node_size)
        for row in range(
                start_wall_top,
                ((self.table_walls_start_y + width) // self.node_size) + 1):
            for node in self.node_matrix[row]:
                node.role = TileRole.OBSTACLE

    def add_bottom_wall(self, width):
        end_wall_bot = min(len(self.node_matrix), (self.table_walls_end_y // self.node_size)+1)
        for row in range(
                (self.table_walls_end_y - width) // self.node_size,
                end_wall_bot):
            for node in self.node_matrix[row]:
                node.role = TileRole.OBSTACLE

    def add_right_wall(self, width):
        end_wall_right = min(len(self.node_matrix[0]), (self.table_walls_end_x//self.node_size)+1)
        for column in range(
                (self.table_walls_end_x - width) // self.node_size,
                end_wall_right):
            for row in range(len(self.node_matrix)):
                node = self.get_node_from_matrix_coordinates((column, row))
                node.role = TileRole.OBSTACLE

    def add_left_wall(self, width):
        start_wall_left = max(0, (self.table_walls_start_x // self.node_size))
        for column in range(
                start_wall_left,
                ((self.table_walls_start_x + width)//self.node_size)+1):
            for row in range(len(self.node_matrix)):
                node = self.get_node_from_matrix_coordinates((column, row))
                node.role = TileRole.OBSTACLE

    def add_table_walls(self):
        width = self.robot_width + self.safety_cushion
        self.add_top_wall(width)
        self.add_bottom_wall(width)
        self.add_left_wall(width)
        self.add_right_wall(width)

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
            self.set_obstacle(node)

    def create_pucks(self):
        """Specifies which nodes should be considered as pucks and then adds their padding."""
        for pixel_position in self.pucks:
            node = self.get_node_from_pixel(pixel_position)
            self.set_puck(node)

    def create_round_obstacle(self, obstacle, radius, role):
        width, height = obstacle.pixel_coordinates_center
        lower_range_column = int(max(0, ((height - radius) // self.node_size)))
        lower_range_row = int(max(0, ((width - radius) // self.node_size)))
        higher_range_column = int(min(len(self.node_matrix), ((height + radius) // self.node_size) + 1))
        higher_range_row = int(min(len(self.node_matrix[0]), ((width + radius) // self.node_size) + 1))

        for column in range(lower_range_column, higher_range_column):
            for row in range(lower_range_row, higher_range_row):
                node = self.get_node_from_matrix_coordinates((row, column))
                distance = get_distance(obstacle.pixel_coordinates_center, node.pixel_coordinates_center)
                if distance < radius:
                    node.role = TileRole.CUSHION
        obstacle.role = role

    def create_square_obstacle(self, obstacle, length, role):
        width, height = obstacle.pixel_coordinates_center
        lower_range_column = int(max(0, ((height - length) // self.node_size)))
        lower_range_row = int(max(0, ((width - length) // self.node_size)))
        higher_range_column = int(min(len(self.node_matrix), ((height + length) // self.node_size)))
        higher_range_row = int(min(len(self.node_matrix[0]), ((width + length) // self.node_size)))

        for column in range(lower_range_column, higher_range_column):
            for row in range(lower_range_row, higher_range_row):
                node = self.get_node_from_matrix_coordinates((row, column))
                node.role = TileRole.CUSHION
        obstacle.role = role

    def create_diagonal_obstacle(self, obstacle, cushion, role):
        obstacle.role = role

        distance = (cushion // self.node_size) + 1
        self.add_cushion(obstacle, distance, TileRole.CUSHION)

    def set_obstacle(self, obstacle):
        if self.obstacle_representation is ObstacleRepresentation.RADIUS:
            self.create_round_obstacle(obstacle, self.obstacle_cushion_width, TileRole.OBSTACLE)

        elif self.obstacle_representation is ObstacleRepresentation.DIAGONAL:
            self.create_diagonal_obstacle(obstacle, self.obstacle_cushion_width, TileRole.OBSTACLE)

        elif self.obstacle_representation is ObstacleRepresentation.SQUARE:
            self.create_square_obstacle(obstacle, self.obstacle_cushion_width, TileRole.OBSTACLE)
        else:
            self.create_round_obstacle(obstacle, self.obstacle_cushion_width, TileRole.OBSTACLE)

    def set_puck(self, puck):
        if self.obstacle_representation is ObstacleRepresentation.RADIUS:
            self.create_round_obstacle(puck, self.obstacle_puck_width, TileRole.PUCK)
        elif self.obstacle_representation is ObstacleRepresentation.DIAGONAL:
            self.create_diagonal_obstacle(puck, self.obstacle_puck_width, TileRole.PUCK)
        elif self.obstacle_representation is ObstacleRepresentation.SQUARE:
            self.create_square_obstacle(puck, self.obstacle_puck_width, TileRole.PUCK)
        else:
            self.create_round_obstacle(puck, self.obstacle_puck_width, TileRole.PUCK)

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

        # distance = (self.obstacle_puck_width // self.node_size) + 1
        # self.add_cushion_in_direction(end, distance, TileRole.END, Direction.DOWN)
        # self.add_cushion_in_direction(end, distance, TileRole.END, Direction.LEFT)
        # self.add_cushion_in_direction(end, distance, TileRole.END, Direction.UP)
        # self.add_cushion_in_direction(end, distance, TileRole.END, Direction.RIGHT)

    def get_start_node(self):
        """Gets the starting node"""
        return self.get_node_from_pixel(self.start_node_location)

    def get_end_node(self):
        """Gets the end node"""
        return self.get_node_from_pixel(self.end_node_location)

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


def get_distance(point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    return math.sqrt(pow(x2-x1, 2) + pow(y2-y1, 2))
