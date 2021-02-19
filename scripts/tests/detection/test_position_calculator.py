from scripts.src.detection.utils.position import Position

from scripts.src.detection.utils.PositionCalculator import PositionCalculator

A_POSITION = Position(0, 10)
A_POSITION_TO_DETECT_ANGLE = Position(20, 30)
ANOTHER_POSITION = Position(10, 30)
ANOTHER_POSITION_TO_DETECT_ANGLE = Position(20, 40)


def test_given_two_position_then_should_return_right_x_position_between_the_two():
    actual_distance_of_x = PositionCalculator.calculate_difference_of_x(A_POSITION,
                                                                        ANOTHER_POSITION)

    assert actual_distance_of_x == 10

def test_given_two_position_then_should_return_right_y_position_between_the_two():
    actual_distance_of_y = PositionCalculator.calculate_difference_of_y(A_POSITION,
                                                                        ANOTHER_POSITION)

    assert actual_distance_of_y == 20

def test_given_two_position_then_should_return_right_angle_between_the_two():
    actual_angle = PositionCalculator.calculate_angle_between_two_position(A_POSITION_TO_DETECT_ANGLE,
                                                                             ANOTHER_POSITION_TO_DETECT_ANGLE)

    assert actual_angle == 90


