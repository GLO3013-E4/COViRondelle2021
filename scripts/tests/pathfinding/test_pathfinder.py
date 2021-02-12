from unittest.mock import Mock
import pytest

from scripts.src.pathfinding.pathfinder import Pathfinder
from scripts.src.pathfinding.node import Node
from scripts.src.pathfinding.path_not_found_exception import PathNotFoundException


class TestPathfinder:
    @classmethod
    def setup_class(cls):
        cls.A_MATRIX_POSITION = (11, 15)
        cls.A_PIXEL_POSITION = (144, 5)
        cls.A_WIDTH = 12
        cls.A_HEIGHT = 9
        cls.A_NODE = Node(cls.A_MATRIX_POSITION, cls.A_PIXEL_POSITION, cls.A_WIDTH, cls.A_HEIGHT)
        cls.ANOTHER_NODE = Node(cls.A_MATRIX_POSITION, cls.A_PIXEL_POSITION,
                                cls.A_WIDTH, cls.A_HEIGHT)
        cls.A_PATH = [cls.A_NODE, cls.ANOTHER_NODE]

    def setup_method(self):
        self._map = Mock()
        self.map_drawer = Mock()
        self.pathfinding_algorithm = Mock()
        self.image = Mock()
        self.pathfinder = Pathfinder(self._map, self.map_drawer, self.pathfinding_algorithm)

    def test_initial_path_is_empty(self):
        assert not self.pathfinder.path

    def test_when_find_square_matrix_path_then_path_is_found(self):
        self._map.get_start_node.return_value = self.A_NODE
        self._map.get_end_node.return_value = self.ANOTHER_NODE

        self.pathfinder.find_square_matrix_path()

        self._map.get_start_node.assert_called_once()
        self.pathfinding_algorithm.find_path.assert_called_once_with(self.A_NODE, self.ANOTHER_NODE)

    def test_given_successful_path_when_find_square_matrix_path_then_update_path(self):
        self.pathfinding_algorithm.find_path.return_value = self.A_PATH

        self.pathfinder.find_square_matrix_path()

        assert self.pathfinder.path == self.A_PATH

    def test_given_unsuccessful_path_when_find_square_matrix_path_then_throw_exception(self):
        self.pathfinding_algorithm.find_path.side_effect = PathNotFoundException()

        with pytest.raises(PathNotFoundException):
            self.pathfinder.find_square_matrix_path()

    def test_when_show_then_draw_image(self):
        self.pathfinder.show()

        self.map_drawer.draw_map.assert_called_once_with(self._map, self.pathfinder.path)

    def test_when_show_then_get_image(self):
        self.pathfinder.show()

        self.map_drawer.get_image.assert_called_once_with()

    def test_when_show_then_show_image(self):
        self.map_drawer.get_image.return_value = self.image

        self.pathfinder.show()

        self.image.show.assert_called_once()