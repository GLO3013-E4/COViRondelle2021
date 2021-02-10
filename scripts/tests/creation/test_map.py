from scripts.creation.Map import Map
from scripts.creation.TileRole import TileRole

from PIL import Image


class TestMap:
    @classmethod
    def setup_class(cls):
        cls.AN_IMAGE_WIDTH = 300
        cls.AN_IMAGE_HEIGHT = 300
        cls.AN_IMAGE = Image.new("RGB", (cls.AN_IMAGE_WIDTH, cls.AN_IMAGE_HEIGHT))
        cls.SOME_OBSTACLES = [
            (0, 0),
            (75, 100)
        ]
        cls.SOME_PUCKS = [
            (110, 23),
            (300, 300)
        ]
        cls.AN_ENDING_POSITION = (40, 60)
        cls.A_STARTING_POSITION = (213, 170)
        cls.A_NODE_SIZE = 25
        cls.A_SAFETY_CUSHION = 20
        cls.A_ROBOT_WIDTH = 100
        cls.AN_OBSTACLE_WIDTH = 40
        cls.A_PUCK_WIDTH = 25
        cls.A_NODE_POSITION = (5, 5)
        cls.A_PIXEL_POSITION = (55, 55)
        cls.A_DISTANCE = 2
        cls.AN_EMPTY_DISTANCE = 0
        cls.A_NEGATIVE_DISTANCE = -1
        cls.A_CUSHION_ROLE = TileRole.CUSHION
        cls.A_MATRIX_POSITION = (8, 6)

        cls.EXPECTED_NUMBER_OF_COLUMNS = 13
        cls.EXPECTED_NUMBER_OF_NODES_PER_ROW = 13
        cls.EXPECTED_NUMBER_OF_TOTAL_NODES = 169
        cls.EXPECTED_NUMBER_OF_NEIGHBORS = {2, 3, 4}
        cls.EXPECTED_NEIGHBOR_DISTANCE = 1
        cls.EXPECTED_MATRIX_POSITION = (6, 8)

    def setup_method(self):
        self.EXPECTED_CUSHION_NODES_POSITION = {
            (5, 3): 0,
            (4, 4): 0,
            (5, 4): 0,
            (6, 4): 0,
            (3, 5): 0,
            (4, 5): 0,
            (5, 5): 0,
            (6, 5): 0,
            (7, 5): 0,
            (4, 6): 0,
            (5, 6): 0,
            (6, 6): 0,
            (5, 7): 0
        }
        self.Map = Map(self.AN_IMAGE, self.SOME_OBSTACLES, self.SOME_PUCKS, self.A_STARTING_POSITION, self.AN_ENDING_POSITION, self.A_NODE_SIZE, self.A_SAFETY_CUSHION, self.A_ROBOT_WIDTH, self.AN_OBSTACLE_WIDTH, self.A_PUCK_WIDTH)

    def test_create_nodes_then_node_matrix_is_not_empty(self):
        self.Map.create_nodes()

        assert self.Map.node_matrix

    def test_create_nodes_then_number_of_nodes_in_each_row_is_as_expected(self):
        self.Map.create_nodes()

        assert len(self.Map.node_matrix[0]) == self.EXPECTED_NUMBER_OF_NODES_PER_ROW

    def test_create_nodes_then_number_of_columns_is_as_expected(self):
        self.Map.create_nodes()

        assert len(self.Map.node_matrix) == self.EXPECTED_NUMBER_OF_COLUMNS

    def test_create_nodes_then_number_of_nodes_created_is_as_expected(self):
        self.Map.create_nodes()

        assert sum([len(row) for row in self.Map.node_matrix]) == self.EXPECTED_NUMBER_OF_TOTAL_NODES

    def test_connect_nodes_then_each_node_has_an_expected_number_of_neighbors(self):
        self.Map.create_nodes()

        self.Map.connect_nodes()

        for line in self.Map.node_matrix:
            for node in line:
                assert len(node.neighbors) in self.EXPECTED_NUMBER_OF_NEIGHBORS

    def test_connect_nodes_then_each_neighbor_is_an_expected_distance_away_from_the_original_node(self):
        self.Map.create_nodes()

        self.Map.connect_nodes()

        for line in self.Map.node_matrix:
            for node in line:
                x1, y1 = node.matrix_center

                for neighbor, angle in node.neighbors:
                    x2, y2 = neighbor.matrix_center
                    distance = abs(x2-x1) + abs(y2-y1)

                    assert distance == self.EXPECTED_NEIGHBOR_DISTANCE

    def test_add_cushion_then_expected_nodes_are_seen_as_cushions(self):
        self.Map.create_nodes()
        self.Map.connect_nodes()
        start_node = self.Map.get_node_from_matrix_coordinates(self.A_NODE_POSITION)
        obstacles_seen = 0

        self.Map.add_cushion(start_node, self.A_DISTANCE, self.A_CUSHION_ROLE)

        for line in self.Map.node_matrix:
            for node in line:
                if node.role is TileRole.CUSHION:
                    obstacles_seen += 1
                    assert node.matrix_center in self.EXPECTED_CUSHION_NODES_POSITION
                    self.EXPECTED_CUSHION_NODES_POSITION[node.matrix_center] += 1
        assert all(seen for seen in self.EXPECTED_CUSHION_NODES_POSITION.values())
        assert sum(self.EXPECTED_CUSHION_NODES_POSITION.values()) == obstacles_seen

    def test_add_cushion_with_zero_distance_then_zero_nodes_are_seen_as_cushions(self):
        self.Map.create_nodes()
        self.Map.connect_nodes()
        node = self.Map.get_node_from_matrix_coordinates(self.A_NODE_POSITION)
        obstacles_seen = 0

        self.Map.add_cushion(node, self.AN_EMPTY_DISTANCE, self.A_CUSHION_ROLE)

        for line in self.Map.node_matrix:
            for node in line:
                if node.role is TileRole.CUSHION:
                    obstacles_seen += 1
        assert not obstacles_seen

    def test_add_cushion_with_negative_distance_then_zero_nodes_are_seen_as_cushions(self):
        self.Map.create_nodes()
        self.Map.connect_nodes()
        node = self.Map.get_node_from_matrix_coordinates(self.A_NODE_POSITION)
        obstacles_seen = 0

        self.Map.add_cushion(node, self.A_NEGATIVE_DISTANCE, self.A_CUSHION_ROLE)

        for line in self.Map.node_matrix:
            for node in line:
                if node.role is TileRole.CUSHION:
                    obstacles_seen += 1
        assert not obstacles_seen

    def test_create_obstacles_then_add_obstacle_role_to_expected_nodes(self):
        self.Map.create_nodes()
        self.Map.connect_nodes()

        self.Map.create_obstacles()

        for obstacle_pixel_position in self.SOME_OBSTACLES:
            node = self.Map.get_node_from_pixel(obstacle_pixel_position)
            assert node.role is TileRole.OBSTACLE

    def test_create_obstacles_without_obstacles_then_there_is_no_obstacle_node(self):
        self.Map.create_nodes()
        self.Map.connect_nodes()
        self.Map.obstacles = []

        self.Map.create_obstacles()

        assert not [node for line in self.Map.node_matrix for node in line if node.role is TileRole.OBSTACLE]

    def test_create_pucks_then_add_puck_role_to_expected_nodes(self):
        self.Map.create_nodes()
        self.Map.connect_nodes()

        self.Map.create_pucks()

        for puck_pixel_position in self.SOME_PUCKS:
            node = self.Map.get_node_from_pixel(puck_pixel_position)
            assert node.role is TileRole.PUCK

    def test_create_pucks_without_pucks_then_there_is_no_obstacle_node(self):
        self.Map.create_nodes()
        self.Map.connect_nodes()
        self.Map.pucks = []

        self.Map.create_pucks()

        assert not [node for line in self.Map.node_matrix for node in line if node.role is TileRole.PUCK]

    def test_create_start_node_then_expected_node_has_start_role(self):
        self.Map.create_nodes()
        self.Map.connect_nodes()

        self.Map.create_start_node()

        assert self.Map.get_node_from_pixel(self.A_STARTING_POSITION).role is TileRole.START

    def test_create_end_node_then_expected_node_has_end_role(self):
        self.Map.create_nodes()
        self.Map.connect_nodes()

        self.Map.create_end_node()

        assert self.Map.get_node_from_pixel(self.AN_ENDING_POSITION).role is TileRole.END

    def test_get_start_node(self):
        self.Map.create_nodes()
        self.Map.create_start_node()

        node = self.Map.get_start_node()

        assert node.role is TileRole.START
        assert self.Map.get_node_from_pixel(self.A_STARTING_POSITION) is node

    def test_get_end_node(self):
        self.Map.create_nodes()
        self.Map.connect_nodes()
        self.Map.create_end_node()

        node = self.Map.get_end_node()

        assert node.role is TileRole.END
        assert self.Map.get_node_from_pixel(self.AN_ENDING_POSITION) is node

    def test_render_map_then_node_matrix_is_not_empty(self):
        self.Map.render_map()

        assert self.Map.node_matrix

    def test_get_node_from_pixel(self):
        self.Map.create_nodes()

        node = self.Map.get_node_from_pixel(self.A_STARTING_POSITION)

        assert node.matrix_center == self.EXPECTED_MATRIX_POSITION

    def test_get_node_from_matrix_coordinates(self):
        self.Map.create_nodes()

        node = self.Map.get_node_from_matrix_coordinates(self.A_MATRIX_POSITION)

        assert node.matrix_center == self.EXPECTED_MATRIX_POSITION
