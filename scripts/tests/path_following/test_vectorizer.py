import math


from scripts.src.path_following.vectorizer import Vectorizer
from scripts.src.path_following.movement_mode import MovementMode
from scripts.src.path_following.config import NODE_SIZE


class TestVectorizer:
    def setup_method(self):
        self.vectorizer = Vectorizer()

    def test_given_perpendicular_nodes_when_vectorize_then_return_correct_vectors(self):
        nodes = [
            (0, 0), (1, 0), (1, -1), (0, -1), (0, 0)
        ]

        vectors = self.vectorizer.vectorize(nodes)

        assert vectors == [[1, 0], [1, math.pi/2], [1, math.pi], [1, -math.pi/2]]

    def test_given_diagonal_nodes_when_vectorize_then_return_correct_vectors(self):
        nodes = [
            (-1, -1), (0, 0), (1, -1), (0, 0), (-1, -1)
        ]

        vectors = self.vectorizer.vectorize(nodes)

        assert vectors == [[math.sqrt(2), -math.pi/4], [math.sqrt(2), math.pi/4], [math.sqrt(2), -math.pi*(3/4)], [math.sqrt(2), math.pi*(3/4)]]

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

        assert corrected_vector == [1, math.pi*(3/4)]

    def test_given_diagonal_vectors_2_when_adjust_vector_angle_from_robot_pov_then_return_correct_angle(self):
        last_vector = (None, math.pi*(1/4))
        vector = (1, -math.pi*(1/4))

        corrected_vector = self.vectorizer.adjust_vector_angle_from_robot_pov(last_vector, vector)

        assert corrected_vector == [1, -math.pi*(1/2)]

    def test_given_two_negative_vectors_when_adjust_vector_angle_from_robot_pov_then_return_correct_angle(self):
        last_vector = (None, -math.pi/2)
        vector = (1, -math.pi*(3/4))

        corrected_vector = self.vectorizer.adjust_vector_angle_from_robot_pov(last_vector, vector)

        assert corrected_vector == [1, -math.pi/4]

    def test_given_two_positive_vectors_when_adjust_vector_angle_from_robot_pov_then_return_correct_angle(self):
        last_vector = (None, 0)
        vector = (1, math.pi/2)

        corrected_vector = self.vectorizer.adjust_vector_angle_from_robot_pov(last_vector, vector)

        assert corrected_vector == [1, math.pi/2]

    def test_given_two_equal_vectors_when_adjust_vector_angle_from_robot_pov_then_return_correct_angle(self):
        last_vector = (None, math.pi)
        vector = (1, math.pi)

        corrected_vector = self.vectorizer.adjust_vector_angle_from_robot_pov(last_vector, vector)

        assert corrected_vector == [1, 0]

    def test_given_two_positive_vectors_where_difference_is_bigger_than_pi_when_adjust_vector_angle_from_robot_pov_then_correction_angle_is_negative(self):
        last_vector = (None, 0)
        vector = (1, math.pi*(3/2))

        corrected_vector = self.vectorizer.adjust_vector_angle_from_robot_pov(last_vector, vector)

        assert corrected_vector == [1, -math.pi/2]

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

    def test_when_minimize_vectors_then_return_correct_vectors(self):
        robot_angle = math.pi*(3/4)
        self.vectorizer.set_robot_angle(robot_angle)
        vectors = [
            (1, -math.pi / 4), (1, -math.pi / 2), (1, -math.pi / 2), (1, math.pi), (1, math.pi),
            (1, -math.pi * (3 / 4)), (1, -math.pi * (3 / 4))
        ]
        adjusted_vectors = self.vectorizer.adjust_vector_angles_from_robot_pov(vectors)

        minimized_vectors = self.vectorizer.minimize_vectors(adjusted_vectors)
        print(minimized_vectors)
        assert minimized_vectors == [
            [1, math.pi, MovementMode.GRIP], [2, -math.pi/4, MovementMode.GRIP], [2, -math.pi/2, MovementMode.GRIP], [2, math.pi/4, MovementMode.GRIP]
        ]

    def test_given_first_vector_is_aligned_when_minimize_vectors_then_return_correct_vectors(self):
        robot_angle = -math.pi/4
        self.vectorizer.set_robot_angle(robot_angle)
        vectors = [
            (1, -math.pi / 4), (1, -math.pi / 2), (1, -math.pi / 2), (1, math.pi), (1, math.pi),
            (1, -math.pi * (3 / 4)), (1, -math.pi * (3 / 4))
        ]
        adjusted_vectors = self.vectorizer.adjust_vector_angles_from_robot_pov(vectors)

        minimized_vectors = self.vectorizer.minimize_vectors(adjusted_vectors)

        assert minimized_vectors == [
            [1, 0, MovementMode.GRIP], [2, -math.pi/4, MovementMode.GRIP], [2, -math.pi/2, MovementMode.GRIP], [2, math.pi/4, MovementMode.GRIP]
        ]

    def test_if_checkpoint_is_None(self):
        robot_position = (-100, -100)
        self.vectorizer.set_robot_position(robot_position)
        nodes = [
            (0, 0), (15, 0), (30, 0)
        ]

        corrected_path = self.vectorizer.get_path_from_robot(nodes)

        assert corrected_path == [(-100, -100), (0, 0), (15, 0), (30, 0)]

    def test_if_robot_is_close_to_path_and_was_updated_recently(self):
        pass

    def test_if_robot_is_close_to_path_but_wasnt_updated_recently(self):
        pass

    def test_if_robot_is_not_close_to_path(self):
        pass

    def test_when_set_goal_if_robot_is_close_to_a_checkpoint_and_last_checkpoint_is_further_then_dont_update_checkpoint(self):
        pass

    def test_when_set_goal_if_robot_is_close_to_a_checkpoint_and_last_checkpoint_is_past_then_update_checkpoint(self):
        pass

    def test_when_set_goal_if_robot_is_far_from_all_checkpoints_then_dont_update(self):
        pass

"""
    def test_given_to_min_is_false_when_path_to_vectors_then_call_right_methods(self):
        robot_position = (15, 0)
        robot_angle = 0
        vectorizer = Vectorizer(minimize=False)
        vectorizer.set_robot_position(robot_position)
        vectorizer.set_robot_angle(robot_angle)
        nodes = [
            (15, 0), (30, 0), (45, 0)
        ]
        vectorizer.set_path(nodes)

        vectors = vectorizer.path_to_vectors()

        assert vectors == [
            [15, 0, MovementMode.GRIP], [15, 0, MovementMode.GRIP]
        ]

    def test_given_mode_ohmmeter_when_path_to_vectors_then_angles_are_adjusted(self):
        robot_position = (15, 0)
        robot_angle = 0
        vectorizer = Vectorizer(minimize=False)
        vectorizer.set_robot_position(robot_position)
        vectorizer.set_robot_angle(robot_angle)
        nodes = [
            (15, 0), (30, 0), (45, 0)
        ]
        vectorizer.set_path(nodes)
        vectorizer.set_mode(MovementMode.OHMMETER)

        vectors = vectorizer.path_to_vectors()

        assert vectors == [
            [15, math.pi/2, MovementMode.OHMMETER], [15, 0, MovementMode.OHMMETER]
        ]

    def test_given_mode_grip_when_path_to_vectors_then_return_grip_mode(self):
        robot_position = (0, 0)
        robot_angle = 0
        vectorizer = Vectorizer(minimize=False)
        vectorizer.set_robot_position(robot_position)
        vectorizer.set_robot_angle(robot_angle)
        nodes = [
            (15, 0), (30, 0), (45, 0)
        ]
        vectorizer.set_path(nodes)

        vectors = vectorizer.path_to_vectors()

        assert all(vector[2] is MovementMode.GRIP for vector in vectors)

    def test_given_ohmmeter_grip_when_path_to_vectors_then_return_ohmmeter_mode(self):
        robot_position = (0, 0)
        robot_angle = 0
        vectorizer = Vectorizer(minimize=False)
        vectorizer.set_robot_position(robot_position)
        vectorizer.set_robot_angle(robot_angle)
        nodes = [
            (15, 0), (30, 0), (45, 0)
        ]
        vectorizer.set_path(nodes)
        vectorizer.set_mode(MovementMode.OHMMETER)

        vectors = vectorizer.path_to_vectors()

        assert all(vector[2] is MovementMode.OHMMETER for vector in vectors)

    def test_given_mode_ohmmeter_when_adjust_vectors_angle_from_robot_pov_then_angles_are_adjusted(self):
        robot_angle = math.pi*(3/4)
        self.vectorizer.set_robot_angle(robot_angle)
        vectors = [
            (1, -math.pi/4), (1, -math.pi/2), (1, -math.pi/2), (1, math.pi), (1, math.pi),
            (1, -math.pi*(3/4)), (1, -math.pi*(3/4))
        ]
        self.vectorizer.set_mode(MovementMode.OHMMETER)

        adjusted_vectors = self.vectorizer.adjust_vector_angles_from_robot_pov(vectors)

        adjusted_angles = [vector[1] for vector in adjusted_vectors]
        assert adjusted_angles == [-math.pi/2, -math.pi/4, 0, -math.pi/2, 0, math.pi/4, 0]

    def test_given_path_when_adjust_vector_angles_from_robot_pov_then_the_angle_between_the_robot_and_its_goal_is_zero(self):
        robot_angle = 0
        self.vectorizer.set_robot_angle(robot_angle)
        self.vectorizer.set_goal((3, 4))
        self.vectorizer.set_path([(0, 0), (3, 3)])
        vectors = [
            (1, -math.pi/4), (1, -math.pi/4), (1, -math.pi/4)
        ]
        self.vectorizer.set_mode(MovementMode.OHMMETER)

        adjusted_vectors = self.vectorizer.adjust_vector_angles_from_robot_pov(vectors)

        adjusted_angles = [vector[1] for vector in adjusted_vectors]
        #assert adjusted_angles == [-math.pi/4, 0, 0, -math.pi/4]
        assert True
"""
#update on set path
# Checker quand path a seulement une node et que c'est exactement le goal ou mettons quand c'est une node mais que t'es a 112 pixels et que t'as le bon angle est-ce que ca sort [(0,0,0)] ?