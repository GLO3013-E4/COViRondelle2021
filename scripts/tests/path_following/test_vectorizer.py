from scripts.src.path_following.vectorizer import Vectorizer
from scripts.src.pathfinding.node import Node


class TestVectorizer:
    def setup_method(self):
        self.vectorizer = Vectorizer()

    def test_given_nodes_in_stairs_pattern_1_when_smooth_path_then_smooth_stairs_into_diagonals(self):
        nodes = [
            Node((0, 0), None, None, None), Node((0, 1), None, None, None), Node((1, 1), None, None, None),
            Node((1, 2), None, None, None), Node((2, 2), None, None, None), Node((2, 3), None, None, None),
            Node((3, 3), None, None, None),
        ]

        smoothed_nodes = self.vectorizer.smooth_path(nodes)

        smoothed_nodes = [node.matrix_center for node in smoothed_nodes]
        assert smoothed_nodes == [
            (0, 0), (1, 1), (2, 2), (3, 3)
        ]

    def test_given_nodes_in_stairs_pattern_2_when_smooth_path_then_smooth_stairs_into_diagonals(self):
        nodes = [
            Node((0, 0), None, None, None), Node((1, 0), None, None, None), Node((1, 1), None, None, None),
            Node((2, 1), None, None, None), Node((2, 2), None, None, None), Node((3, 2), None, None, None),
            Node((3, 3), None, None, None),
        ]

        smoothed_nodes = self.vectorizer.smooth_path(nodes)

        smoothed_nodes = [node.matrix_center for node in smoothed_nodes]
        assert smoothed_nodes == [
            (0, 0), (1, 1), (2, 2), (3, 3)
        ]

    def test_given_nodes_in_stairs_pattern_3_when_smooth_path_then_smooth_stairs_into_diagonals(self):
        nodes = [
            Node((-3, 0), None, None, None), Node((-2, 0), None, None, None), Node((-1, 0), None, None, None),
            Node((0, 0), None, None, None), Node((1, 0), None, None, None), Node((1, 1), None, None, None),
            Node((2, 1), None, None, None), Node((2, 2), None, None, None), Node((3, 2), None, None, None),
            Node((3, 3), None, None, None), Node((3, 4), None, None, None), Node((3, 5), None, None, None)
        ]

        smoothed_nodes = self.vectorizer.smooth_path(nodes)

        smoothed_nodes = [node.matrix_center for node in smoothed_nodes]

        assert smoothed_nodes == [
            (-3, 0), (-2, 0), (-1, 0), (0, 0), (1, 1), (2, 2), (3, 3), (3, 4), (3, 5)
        ]

    def test_given_nodes_in_stairs_pattern_4_when_smooth_path_then_smooth_stairs_into_diagonals(self):
        nodes = [
            Node((0, 0), None, None, None), Node((1, 0), None, None, None), Node((1, 1), None, None, None),
            Node((2, 1), None, None, None), Node((3, 1), None, None, None), Node((4, 1), None, None, None),
            Node((4, 2), None, None, None)
        ]

        smoothed_nodes = self.vectorizer.smooth_path(nodes)

        smoothed_nodes = [node.matrix_center for node in smoothed_nodes]

        assert smoothed_nodes == [
            (0, 0), (1, 1), (2, 1), (3, 1), (4, 2)
        ]



