from Node import Node


#TODO: ajouter le drawer
#TODO: ajouter le gérage des pucks
#TODO: ajuster si la destination est une puck et que c'est vu un peu comme un obstacle
#TODO: (l'algo ne réussira jamais à se rendre à la node sans ajustements)
#TODO: what if l'algo voit que le start/end est dans un obstacle
#TODO: what if les seules places que le robot peut se déplacer c'est dans un "obstacle" (peut-être juste ajuster les cushions)


class Map:
    def __init__(self, image, node_size, safety_cushion, robot_width, obstacle_width, puck_width, obstacle_cushion_width, obstacles, pucks, obstacle_puck_width, start, end):
        self.node_size = node_size
        self.safety_cushion = safety_cushion
        self.robot_width = robot_width
        self.obstacle_width = obstacle_width
        self.puck_width = puck_width
        self.obstacle_cushion_width = obstacle_cushion_width
        self.obstacle_puck_width = obstacle_puck_width

        self.image = image
        self.width, self.height = self.image.size

        self.obstacles = obstacles
        self.pucks = pucks
        self.start_node_location = start
        self.end_node_location = end

        self.node_matrix = self.create_nodes()
        self.connect_nodes()
        self.create_obstacles()
        self.create_pucks()
        self.create_start_node()
        self.create_end_node()

    # get node matrix? et l'objet manipule la matrice au lieu que ce soit static-ish?
    def create_nodes(self):
        node_matrix = [
            [] for _ in range((self.height // self.node_size) + 1)
        ]

        for i in range((self.height // self.node_size)+1):
            for j in range((self.width // self.node_size)+1):
                y = i * self.node_size + self.node_size / 2
                x = j * self.node_size + self.node_size / 2
                node_matrix[i].append(Node((i, j), (x, y), self.node_size, self.node_size))

        return node_matrix

    def connect_nodes(self):
        for i, line in enumerate(self.node_matrix):
            for j, node in enumerate(line):
                possible_neighbors = [
                    (y, x)

                    for (y, x) in
                    [(node.matrix_center[0] - 1, node.matrix_center[1]), (node.matrix_center[0] + 1, node.matrix_center[1]),
                     (node.matrix_center[0], node.matrix_center[1] - 1), (node.matrix_center[0], node.matrix_center[1] + 1)]

                    if (x >= 0 and x < len(self.node_matrix[0]) and y >= 0 and y < len(self.node_matrix) and (
                    x, y) != node.matrix_center)
                ]

                for (y, x) in possible_neighbors:
                    node.neighbors.append(self.node_matrix[y][x])

    def add_cushion(self, node, distance):
        if distance > 0:
            for neighbor in node.neighbors:
                if neighbor.role == "empty":
                    neighbor.role = "obstacle_cushion"
                self.add_cushion(neighbor, distance - 1)

    def create_obstacles(self):
        for (x, y) in self.obstacles:
            node = self.node_matrix[y // self.node_size][x // self.node_size]
            node.role = "obstacle"

            # add cushion
            distance = (self.obstacle_cushion_width // self.node_size) + 1
            # distance = 0
            self.add_cushion(node, distance)

    def create_pucks(self):
        for (x, y) in self.pucks:
            node = self.node_matrix[y // self.node_size][x // self.node_size]
            node.role = "puck"

            # add cushion
            distance = (self.obstacle_puck_width // self.node_size) + 1
            # distance = 0
            self.add_cushion(node, distance)

    def create_start_node(self):
        start = self.get_start_node()
        start.role = "start"

    def create_end_node(self):
        end = self.get_end_node()
        end.role = "end"

    def get_start_node(self):
        return self.node_matrix[self.start_node_location[1]//self.node_size][self.start_node_location[0]//self.node_size]

    def get_end_node(self):
        return self.node_matrix[self.end_node_location[1] // self.node_size][self.end_node_location[0] // self.node_size]

    def get_image(self):
        return self.image

    def get_node_matrix(self):
        return self.node_matrix
