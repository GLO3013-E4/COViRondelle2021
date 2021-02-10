from scripts.creation.PathfindingAlgorithmFactory import PathfindingAlgorithmFactory
from scripts.creation.BFS import BFS

import pytest


class TestPathfindingAlgorithmFactory:
    @classmethod
    def setup_class(cls):
        cls.BFS_ALGORITHM_NAME = "BFS"
        cls.BFS_CLASS = BFS
        cls.NOT_IMPLEMENTED_ALGORITHM_NAME = "whatever"

    def setup_method(self):
        self.pathfinding_algorithm_factory = PathfindingAlgorithmFactory()

    def test_create_with_bfs_algorithm_then_return_bfs_object(self):
        algorithm = self.pathfinding_algorithm_factory.create(self.BFS_ALGORITHM_NAME)

        assert isinstance(algorithm, self.BFS_CLASS)

    def test_create_with_not_implemented_algorithm_then_raise_exception(self):
        with pytest.raises(Exception):
            self.pathfinding_algorithm_factory.create(self.NOT_IMPLEMENTED_ALGORITHM_NAME)
