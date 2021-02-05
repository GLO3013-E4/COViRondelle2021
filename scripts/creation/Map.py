from PIL import Image, ImageDraw

from params import NODE_SIZE, SAFETY_CUSHION, ROBOT_WIDTH, OBSTACLE_WIDTH, PUCK_WIDTH, OBSTACLE_CUSHION_WIDTH
from Node import Node


#TODO: ajuster si la destination est une puck et que c'est vu un peu comme un obstacle
#TODO: (l'algo ne réussira jamais à se rendre à la node sans ajustements)

class Map:
    def __init__(self, image, node_size, safety_cushion, robot_width, obstacle_width, puck_width, obstacle_cushion_width):
        self.node_size = node_size
        self.safety_cushion = safety_cushion
        self.robot_width = robot_width
        self.obstacle_width = obstacle_width
        self.puck_width = puck_width
        self.obstacle_cushion_width = obstacle_cushion_width

        self.image = image
        self.width, self.height = self.image.size
        self.draw = ImageDraw.Draw(self.image)

        self.node_matrix = self.create_nodes()
        self.connect_nodes()

    # get node matrix? et l'objet manipule la matrice au lieu que ce soit static-ish?
    def create_nodes(self):
        node_matrix = [
            [] for _ in range(self.height // self.node_size)
        ]

        for i in range(self.height // self.node_size):
            for j in range(self.width // self.node_size):
                x = i * self.node_size + self.node_size / 2
                y = j * self.node_size + self.node_size / 2
                node_matrix[i].append(Node((i, j), (x, y), self.node_size, self.node_size))

        return node_matrix

    def connect_nodes(self):
        for i, line in enumerate(self.node_matrix):
            for j, node in enumerate(line):
                possible_neighbors = [
                    (x, y)

                    for (x, y) in
                    [(node.matrix_center[0] - 1, node.matrix_center[1]), (node.matrix_center[0] + 1, node.matrix_center[1]),
                     (node.matrix_center[0], node.matrix_center[1] - 1), (node.matrix_center[0], node.matrix_center[1] + 1)]

                    if (x >= 0 and x < len(self.node_matrix) and y >= 0 and y < len(self.node_matrix[1]) and (
                    x, y) != node.matrix_center)
                ]

                for (y, x) in possible_neighbors:
                    node.neighbors.append(self.node_matrix[y][x])

    def add_cushion(self, node, distance):
        if distance > 0:
            for neighbor in node.neighbors:
                if neighbor.role == "empty":
                    neighbor.role = "obstacle_cushion"

                    # draw cushion
                    #y1, x1 = neighbor.pixel_coordinates_center
                    #draw.rectangle([(x1 - square_identifier_width, y1 - square_identifier_width),
                    #                (x1 + square_identifier_width, y1 + square_identifier_width)], fill=(0, 255, 0))

                self.add_cushion(neighbor, distance - 1)

    def create_obstacles(self):
        # add obstacles
        for (y, x) in obstacles:
            node = node_matrix[x // space_between_squares][y // space_between_squares]
            node.role = "obstacle"

            # add cushion
            distance = (obstacle_cushion // space_between_squares) + 1
            # distance = 0
            recursive_cushion(node, distance)

            # draw obstacle
            #y1, x1 = node.pixel_coordinates_center
            #draw.rectangle([(x1 - square_identifier_width, y1 - square_identifier_width),
            #                (x1 + square_identifier_width, y1 + square_identifier_width)], fill=(255, 255, 255))

    def create_pucks(self):
        pass

    def get_start_node(self):
        start_node = node_matrix[start_node_pixel_center[0]//space_between_squares][start_node_pixel_center[1]//space_between_squares]
        return start_node

    def get_end_node(self):
        end_node = node_matrix[end_node_pixel_center[0] // space_between_squares][
            end_node_pixel_center[1] // space_between_squares]
        return end_node

    # Draw
    #y1, x1 = start_node.pixel_coordinates_center
    #y2, x2 = end_node.pixel_coordinates_center
    #draw.rectangle([(x1 - square_identifier_width, y1 - square_identifier_width),
    #                (x1 + square_identifier_width, y1 + square_identifier_width)], fill=200)
    #draw.rectangle([(x2 - square_identifier_width, y2 - square_identifier_width),
    #                (x2 + square_identifier_width, y2 + square_identifier_width)], fill=120)

    def get_image(self):
        return self.image

    ## draw path
    #for node in path:
    #    y, x = node.pixel_coordinates_center
    #    draw.rectangle([(x - square_identifier_width, y - square_identifier_width),
    #                    (x + square_identifier_width, y + square_identifier_width)], outline=300)


