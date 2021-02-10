import pytest
from unittest.mock import Mock

from scripts.creation.Pathfinder import Pathfinder
from scripts.creation.Node import Node
from scripts.creation.PathNotFoundException import PathNotFoundException


class TestPathfinder:
    @classmethod
    def setup_class(cls):
        cls.A_MATRIX_POSITION = (11, 15)
        cls.A_PIXEL_POSITION = (144, 5)
        cls.A_WIDTH = 12
        cls.A_HEIGHT = 9
        cls.A_NODE = Node(cls.A_MATRIX_POSITION, cls.A_PIXEL_POSITION, cls.A_WIDTH, cls.A_HEIGHT)
        cls.ANOTHER_NODE = Node(cls.A_MATRIX_POSITION, cls.A_PIXEL_POSITION, cls.A_WIDTH, cls.A_HEIGHT)
        cls.A_PATH = [cls.A_NODE, cls.ANOTHER_NODE]

    def setup_method(self):
        self.Map = Mock()
        self.map_drawer = Mock()
        self.pathfinding_algorithm = Mock()
        self.image = Mock()
        self.pathfinder = Pathfinder(self.Map, self.map_drawer, self.pathfinding_algorithm)

    def test_initial_path_is_empty(self):
        assert not self.pathfinder.path

    def test_find_square_matrix_path_then_path_is_found(self):
        self.Map.get_start_node.return_value = self.A_NODE

        self.pathfinder.find_square_matrix_path()

        self.Map.get_start_node.assert_called_once()
        self.pathfinding_algorithm.find_path.assert_called_once_with(self.A_NODE)

    def test_find_square_matrix_path_successful_then_update_path(self):
        self.pathfinding_algorithm.find_path.return_value = self.A_PATH

        self.pathfinder.find_square_matrix_path()

        assert self.pathfinder.path == self.A_PATH

    def test_find_square_matrix_path_unsuccessful_then_throw_exception(self):
        self.pathfinding_algorithm.find_path.side_effect = PathNotFoundException()

        with pytest.raises(PathNotFoundException):
            self.pathfinder.find_square_matrix_path()

    def test_show_then_draw_image(self):
        self.pathfinder.show()

        self.map_drawer.draw_map.assert_called_once_with(self.Map, self.pathfinder.path)

    def test_show_then_get_image(self):
        self.pathfinder.show()

        self.map_drawer.get_image.assert_called_once_with()

    def test_show_then_show_image(self):
        self.map_drawer.get_image.return_value = self.image

        self.pathfinder.show()

        self.image.show.assert_called_once()
