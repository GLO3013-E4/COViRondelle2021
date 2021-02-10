import pytest

from scripts.creation.Node import Node
from scripts.creation.TileRole import TileRole
from scripts.creation.BFS import BFS
from scripts.creation.PathNotFoundException import PathNotFoundException


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

    def setup_method(self):
        self.BFS = BFS()

    def test_given_unconnected_graph_without_end_role_then_raise_path_does_not_exist(self):
        start_node = self.given_unconnected_graph_without_end_role()

        with pytest.raises(PathNotFoundException):
            self.BFS.find_path(start_node)

    def test_given_connected_graph_with_end_role_then_path_is_valid(self):
        start_node = self.given_connected_graph_with_end_role()

        path = self.BFS.find_path(start_node)

        assert path

    def test_given_connected_graph_without_end_role_then_raise_path_does_not_exist(self):
        start_node = self.given_connected_graph_without_end_role()

        with pytest.raises(PathNotFoundException):
            self.BFS.find_path(start_node)

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
