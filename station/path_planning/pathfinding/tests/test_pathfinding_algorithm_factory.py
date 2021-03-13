import pytest

from pathfinding.pathfinding_algorithm_factory import PathfindingAlgorithmFactory
from pathfinding.breadth_first_search import BreadthFirstSearch


class TestPathfindingAlgorithmFactory:
    """Test path-finding algorithm factory"""
    @classmethod
    def setup_class(cls):
        cls.BREADTH_FIRST_SEARCH_ALGORITHM_NAME = "BreadthFirstSearch"
        cls.BREADTH_FIRST_SEARCH_CLASS = BreadthFirstSearch
        cls.NOT_IMPLEMENTED_ALGORITHM_NAME = "whatever"

    def setup_method(self):
        self.pathfinding_algorithm_factory = PathfindingAlgorithmFactory()

    def test_when_create_with_bfs_algorithm_then_return_bfs_object(self):
        algorithm = self.pathfinding_algorithm_factory.create(
            self.BREADTH_FIRST_SEARCH_ALGORITHM_NAME)

        assert isinstance(algorithm, self.BREADTH_FIRST_SEARCH_CLASS)

    def test_when_create_with_not_implemented_algorithm_then_raise_exception(self):
        with pytest.raises(Exception):
            self.pathfinding_algorithm_factory.create(self.NOT_IMPLEMENTED_ALGORITHM_NAME)
