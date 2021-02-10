#test add cushion
#test create obstacles
#test create pucks
#test create start_node
#test create end_node
#test create add pickup cushion
#test get start node
#test get end node
#test render map


# MOCKS MOCKS MOCKS

from scripts.creation.Map import Map

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

        cls.EXPECTED_NUMBER_OF_COLUMNS = 13
        cls.EXPECTED_NUMBER_OF_NODES_PER_ROW = 13
        cls.EXPECTED_NUMBER_OF_TOTAL_NODES = 169
        cls.EXPECTED_NUMBER_OF_NEIGHBORS = {2, 3, 4}
        cls.EXPECTED_NEIGHBOR_DISTANCE = 1

    def setup_method(self):
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










