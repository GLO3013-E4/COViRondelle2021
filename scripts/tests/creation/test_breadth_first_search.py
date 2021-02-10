import pytest

from scripts.creation.node import Node
from scripts.creation.tile_role import TileRole
from scripts.creation.breadth_first_search import BreadthFirstSearch
from scripts.creation.path_not_found_exception import PathNotFoundException


class TestBFS:
    @classmethod
    def setup_class(cls):
        cls.A_WIDTH = 2
        cls.A_HEIGHT = 3
        cls.SOME_MATRIX_COORDINATES = (4, 5)
        cls.SOME_OTHER_MATRIX_COORDINATES = (6, 7)
        cls.SOME_PIXEL_COORDINATES = (8, 9)
        cls.SOME_OTHER_PIXEL_COORDINATES = (10, 11)
        cls.SOME_ANGLE = "40"
        cls.A_MATRIX_POSITION = (11, 15)
        cls.A_PIXEL_POSITION = (144, 5)
        cls.A_WIDTH = 12
        cls.A_HEIGHT = 9
        cls.SOME_NODE = Node(cls.A_MATRIX_POSITION, cls.A_PIXEL_POSITION, cls.A_WIDTH, cls.A_HEIGHT)

    def setup_method(self):
        self.BFS = BreadthFirstSearch()

    def test_given_unconnected_graph_without_end_role_when_find_path_then_raise_path_does_not_exist(self):
        start_node = self.given_unconnected_graph_without_end_role()

        with pytest.raises(PathNotFoundException):
            self.BFS.find_path(start_node, self.SOME_NODE)

    def test_given_connected_graph_with_end_role_when_find_path_then_path_is_valid(self):
        start_node = self.given_connected_graph_with_end_role()

        path = self.BFS.find_path(start_node, self.SOME_NODE)

        assert path

    def test_given_connected_graph_without_end_role_when_find_path_then_raise_path_does_not_exist(self):
        start_node = self.given_connected_graph_without_end_role()

        with pytest.raises(PathNotFoundException):
            self.BFS.find_path(start_node, self.SOME_NODE)

    def given_unconnected_graph_without_end_role(self):
        start_node = Node(self.SOME_MATRIX_COORDINATES, self.SOME_PIXEL_COORDINATES, self.A_WIDTH, self.A_HEIGHT)
        return start_node

    def given_connected_graph_with_end_role(self):
        start_node = Node(self.SOME_MATRIX_COORDINATES, self.SOME_PIXEL_COORDINATES, self.A_WIDTH, self.A_HEIGHT)
        end_node = Node(self.SOME_OTHER_MATRIX_COORDINATES, self.SOME_OTHER_PIXEL_COORDINATES, self.A_WIDTH, self.A_HEIGHT)
        start_node.neighbors.append((end_node, self.SOME_ANGLE))
        end_node.role = TileRole.END
        return start_node

    def given_connected_graph_without_end_role(self):
        start_node = Node(self.SOME_MATRIX_COORDINATES, self.SOME_PIXEL_COORDINATES, self.A_WIDTH, self.A_HEIGHT)
        end_node = Node(self.SOME_OTHER_MATRIX_COORDINATES, self.SOME_OTHER_PIXEL_COORDINATES, self.A_WIDTH, self.A_HEIGHT)
        start_node.neighbors.append((end_node, self.SOME_ANGLE))
        return start_node
