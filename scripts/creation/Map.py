from scripts.creation.Node import Node
from scripts.creation.TileRole import TileRole


#TODO: what if l'algo voit que le start/end est dans un obstacle
#TODO: what if les seules places que le robot peut se déplacer c'est dans un "obstacle" (peut-être juste ajuster les cushions)


class Map:
    def __init__(self, image, obstacles, pucks, start, end, node_size=25, safety_cushion=0, robot_width=100, obstacle_width=40, puck_width=25):
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
        self.create_nodes()
        self.connect_nodes()
        self.create_obstacles()
        self.create_pucks()
        self.create_start_node()
        self.create_end_node()

    def create_nodes(self):
        node_matrix = [
            [] for _ in range((self.height // self.node_size) + 1)
        ]

        for i in range((self.height // self.node_size)+1):
            for j in range((self.width // self.node_size)+1):
                y = i * self.node_size + self.node_size / 2
                x = j * self.node_size + self.node_size / 2
                node_matrix[i].append(Node((i, j), (x, y), self.node_size, self.node_size))

        self.node_matrix = node_matrix

    def connect_nodes(self):
        for i, line in enumerate(self.node_matrix):
            for j, node in enumerate(line):
                possible_neighbors = [
                    (y, x, z)

                    for (y, x, z) in
                    [
                        (node.matrix_center[0] - 1, node.matrix_center[1], "90"),
                        (node.matrix_center[0] + 1, node.matrix_center[1], "270"),
                        (node.matrix_center[0], node.matrix_center[1] - 1, "180"),
                        (node.matrix_center[0], node.matrix_center[1] + 1, "0"),

                        #(node.matrix_center[0] - 1, node.matrix_center[1] - 1, "135"),
                        #(node.matrix_center[0] - 1, node.matrix_center[1] + 1, "45"),
                        #(node.matrix_center[0] + 1, node.matrix_center[1] - 1, "225"),
                        #(node.matrix_center[0] + 1, node.matrix_center[1] + 1, "315"),
                    ]

                    if (x >= 0 and x < len(self.node_matrix[0]) and y >= 0 and y < len(self.node_matrix) and (
                    x, y) != node.matrix_center)
                ]

                for (y, x, z) in possible_neighbors:
                    node.neighbors.append((self.node_matrix[y][x], z))

    def add_cushion(self, node, distance, role):
        if distance > 0:
            for neighbor, angle in node.neighbors:
                if neighbor.role is TileRole.EMPTY:
                    neighbor.role = role
                self.add_cushion(neighbor, distance - 1, role)

    def create_obstacles(self):
        for (x, y) in self.obstacles:
            node = self.node_matrix[y // self.node_size][x // self.node_size]
            node.role = TileRole.OBSTACLE

            distance = (self.obstacle_cushion_width // self.node_size) + 1
            self.add_cushion(node, distance, TileRole.CUSHION)

    def create_pucks(self):
        for (x, y) in self.pucks:
            node = self.node_matrix[y // self.node_size][x // self.node_size]
            node.role = TileRole.PUCK

            distance = (self.obstacle_puck_width // self.node_size) + 1
            self.add_cushion(node, distance, TileRole.CUSHION)

    def create_start_node(self):
        start = self.get_start_node()
        start.role = TileRole.START

    def create_end_node(self):
        end = self.get_end_node()
        end.role = TileRole.END

        distance = (self.obstacle_puck_width // self.node_size) + 1
        self.add_cushion(end, distance, TileRole.END)

    def get_start_node(self):
        return self.get_node_from_pixel(self.start_node_location)

    def get_end_node(self):
        return self.get_node_from_pixel(self.end_node_location)

    def get_image(self):
        return self.image

    def get_node_matrix(self):
        return self.node_matrix

    def get_node_from_pixel(self, pixel):
        x = pixel[0] // self.node_size
        y = pixel[1] // self.node_size
        return self.node_matrix[y][x]

    def get_node_from_matrix_coordinates(self, coordinates):
        x, y = coordinates
        return self.node_matrix[y][x]
