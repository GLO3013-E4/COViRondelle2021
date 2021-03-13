import math

from scripts.src.path_following.vectorizer import Vectorizer
from scripts.src.pathfinding.node import Node


class TestVectorizer:
    def setup_method(self):
        self.vectorizer = Vectorizer()

    def test_given_nodes_in_stairs_under_pattern_when_smooth_path_then_smooth_stairs_into_diagonals(self):
        nodes = [
            Node((0, 0), None, None, None), Node((0, 1), None, None, None),
            Node((1, 1), None, None, None), Node((1, 2), None, None, None),
            Node((2, 2), None, None, None), Node((2, 3), None, None, None),
            Node((3, 3), None, None, None),
        ]

        smoothed_nodes = self.vectorizer.smooth_path(nodes)

        smoothed_nodes = [node.matrix_center for node in smoothed_nodes]
        assert smoothed_nodes == [
            (0, 0), (1, 1), (2, 2), (3, 3)
        ]

    def test_given_nodes_in_stairs_over_pattern_when_smooth_path_then_smooth_stairs_into_diagonals(self):
        nodes = [
            Node((0, 0), None, None, None), Node((1, 0), None, None, None),
            Node((1, 1), None, None, None), Node((2, 1), None, None, None),
            Node((2, 2), None, None, None), Node((3, 2), None, None, None),
            Node((3, 3), None, None, None),
        ]

        smoothed_nodes = self.vectorizer.smooth_path(nodes)

        smoothed_nodes = [node.matrix_center for node in smoothed_nodes]
        assert smoothed_nodes == [
            (0, 0), (1, 1), (2, 2), (3, 3)
        ]

    def test_given_nodes_in_line_stairs_line_pattern_when_smooth_path_then_smooth_stairs_into_diagonals(self):
        nodes = [
            Node((-3, 0), None, None, None), Node((-2, 0), None, None, None),
            Node((-1, 0), None, None, None), Node((0, 0), None, None, None),
            Node((1, 0), None, None, None), Node((1, 1), None, None, None),
            Node((2, 1), None, None, None), Node((2, 2), None, None, None),
            Node((3, 2), None, None, None), Node((3, 3), None, None, None),
            Node((3, 4), None, None, None), Node((3, 5), None, None, None)
        ]

        smoothed_nodes = self.vectorizer.smooth_path(nodes)

        smoothed_nodes = [node.matrix_center for node in smoothed_nodes]

        assert smoothed_nodes == [
            (-3, 0), (-2, 0), (-1, 0), (0, 0), (1, 1), (2, 2), (3, 3), (3, 4), (3, 5)
        ]

    def test_given_nodes_in_stairs_line_stairs_pattern_when_smooth_path_then_smooth_stairs_into_diagonals(self):
        nodes = [
            Node((0, 0), None, None, None), Node((1, 0), None, None, None),
            Node((1, 1), None, None, None), Node((2, 1), None, None, None),
            Node((3, 1), None, None, None), Node((4, 1), None, None, None),
            Node((4, 2), None, None, None)
        ]

        smoothed_nodes = self.vectorizer.smooth_path(nodes)

        smoothed_nodes = [node.matrix_center for node in smoothed_nodes]

        assert smoothed_nodes == [
            (0, 0), (1, 1), (2, 1), (3, 1), (4, 2)
        ]

    def test_given_perpendicular_nodes_when_vectorize_then_return_correct_vectors(self):
        nodes = [
            (0, 0), (1, 0), (1, -1), (0, -1), (0, 0)
        ]

        vectors = self.vectorizer.vectorize(nodes)

        assert vectors == [(1, 0), (1, math.pi/2), (1, math.pi), (1, -math.pi/2)]

    def test_given_diagonal_nodes_when_vectorize_then_return_correct_vectors(self):
        nodes = [
            (-1, -1), (0, 0), (1, -1), (0, 0), (-1, -1)
        ]

        vectors = self.vectorizer.vectorize(nodes)

        assert vectors == [(math.sqrt(2), -math.pi/4), (math.sqrt(2), math.pi/4), (math.sqrt(2), -math.pi*(3/4)), (math.sqrt(2), math.pi*(3/4))]

    def test_given_nodes_with_magnitude_bigger_than_1_when_vectorize_then_return_correct_vectors(self):
        nodes = [
            (0, 0), (3, 4)
        ]

        vectors = self.vectorizer.vectorize(nodes)

        vector = vectors[0]
        assert vector[0] == 5

    def test_given_diagonal_vectors_1_when_adjust_first_vector_angle_from_robot_pov_then_return_correct_angle(self):
        last_vector = (None, math.pi*(3/4))
        vector = (1, -math.pi/2)

        corrected_vector = self.vectorizer.adjust_vector_angle_from_robot_pov(last_vector, vector)

        assert corrected_vector == (1, math.pi*(3/4))

    def test_given_diagonal_vectors_2_when_adjust_vector_angle_from_robot_pov_then_return_correct_angle(self):
        last_vector = (None, math.pi*(1/4))
        vector = (1, -math.pi*(1/4))

        corrected_vector = self.vectorizer.adjust_vector_angle_from_robot_pov(last_vector, vector)

        assert corrected_vector == (1, -math.pi*(1/2))

    def test_given_two_negative_vectors_when_adjust_vector_angle_from_robot_pov_then_return_correct_angle(self):
        last_vector = (None, -math.pi/2)
        vector = (1, -math.pi*(3/4))

        corrected_vector = self.vectorizer.adjust_vector_angle_from_robot_pov(last_vector, vector)

        assert corrected_vector == (1, -math.pi/4)

    def test_given_two_positive_vectors_when_adjust_vector_angle_from_robot_pov_then_return_correct_angle(self):
        last_vector = (None, 0)
        vector = (1, math.pi/2)

        corrected_vector = self.vectorizer.adjust_vector_angle_from_robot_pov(last_vector, vector)

        assert corrected_vector == (1, math.pi/2)

    def test_given_two_equal_vectors_when_adjust_vector_angle_from_robot_pov_then_return_correct_angle(self):
        last_vector = (None, math.pi)
        vector = (1, math.pi)

        corrected_vector = self.vectorizer.adjust_vector_angle_from_robot_pov(last_vector, vector)

        assert corrected_vector == (1, 0)

    def test_given_two_positive_vectors_where_difference_is_bigger_than_pi_when_adjust_vector_angle_from_robot_pov_then_correction_angle_is_negative(self):
        last_vector = (None, 0)
        vector = (1, math.pi*(3/2))

        corrected_vector = self.vectorizer.adjust_vector_angle_from_robot_pov(last_vector, vector)

        assert corrected_vector == (1, -math.pi/2)

    def test_given_everything_aligned_when_adjust_vector_angles_from_robot_pov_then_return_correct_vectors(self):
        robot_angle = 0
        self.vectorizer.set_robot_angle(robot_angle)
        vectors = [
            (1, 0), (1, 0), (1, 0)
        ]

        adjusted_vectors = self.vectorizer.adjust_vector_angles_from_robot_pov(vectors)

        adjusted_angles = [vector[1] for vector in adjusted_vectors]
        assert adjusted_angles == [0, 0, 0]

    def test_given_everything_aligned_but_robot_when_adjust_vector_angles_from_robot_pov_then_return_correct_vectors(self):
        robot_angle = math.pi/2
        self.vectorizer.set_robot_angle(robot_angle)
        vectors = [
            (1, 0), (1, 0), (1, 0)
        ]

        adjusted_vectors = self.vectorizer.adjust_vector_angles_from_robot_pov(vectors)

        adjusted_angles = [vector[1] for vector in adjusted_vectors]
        assert adjusted_angles == [-math.pi/2, 0, 0]

    def test_given_complex_alignment_when_adjust_vector_angles_from_robot_pov_then_return_correct_vectors(self):
        robot_angle = math.pi*(3/4)
        self.vectorizer.set_robot_angle(robot_angle)
        vectors = [
            (1, -math.pi/4), (1, -math.pi/2), (1, -math.pi/2), (1, math.pi), (1, math.pi),
            (1, -math.pi*(3/4)), (1, -math.pi*(3/4))
        ]

        adjusted_vectors = self.vectorizer.adjust_vector_angles_from_robot_pov(vectors)

        adjusted_angles = [vector[1] for vector in adjusted_vectors]
        assert adjusted_angles == [math.pi, -math.pi/4, 0, -math.pi/2, 0, math.pi/4, 0]

