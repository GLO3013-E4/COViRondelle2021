from scripts.src.detection.lower_boundary import LowerBoundary
from scripts.src.mapping.color import Color

RED_COLOR = Color["RED"]
BROWN_COLOR = Color["BROWN"]
GREEN_COLOR = Color["GREEN"]
NOT_A_COLOR = "Uncle"

lower_boundary = LowerBoundary()


def test_given_red_color_then_should_return_right_lower_boundary():
    expected_boundaries = [0, 216, 92]

    actual_boundaries = lower_boundary.get_lower_boundaries(RED_COLOR)

    assert len(actual_boundaries) == len(expected_boundaries)
    assert all([a == b for a, b in zip(actual_boundaries, expected_boundaries)])


def test_given_brown_color_then_should_return_right_lower_boundary():
    expected_boundaries = [0, 121, 15]

    actual_boundaries = lower_boundary.get_lower_boundaries(BROWN_COLOR)

    assert len(actual_boundaries) == len(expected_boundaries)
    assert all([a == b for a, b in zip(actual_boundaries, expected_boundaries)])


def test_given_green_color_then_should_return_right_lower_boundary():
    expected_boundaries = [47, 89, 0]

    actual_boundaries= lower_boundary.get_lower_boundaries(GREEN_COLOR)

    assert len(actual_boundaries) == len(expected_boundaries)
    assert all([a == b for a, b in zip(actual_boundaries, expected_boundaries)])


def test_given_not_a_valid_color_then_should_return_array_of_zeros():
    expected_boundaries = [0, 0, 0]

    actual_boundaries = lower_boundary.get_lower_boundaries(NOT_A_COLOR)

    assert len(actual_boundaries) == len(expected_boundaries)
    assert all([a == b for a, b in zip(actual_boundaries, expected_boundaries)])
