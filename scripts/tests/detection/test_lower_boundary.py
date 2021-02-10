from scripts.src.detection.lower_boundary import LowerBoundary

lower_boundary = LowerBoundary()
red_color = "red"
brown_color = "brown"
green_color = "green"
not_a_color = "Uncle"


def test_given_red_color_then_should_return_right_lower_boundary():
    expected_result = [0, 216, 92]

    actual_result = lower_boundary.get_lower_boundaries(red_color)

    assert expected_result[0] == actual_result[0]
    assert expected_result[1] == actual_result[1]
    assert expected_result[2] == actual_result[2]


def test_given_brown_color_then_should_return_right_lower_boundary():
    expected_result = [0, 121, 15]

    actual_result = lower_boundary.get_lower_boundaries(brown_color)

    assert expected_result[0] == actual_result[0]
    assert expected_result[1] == actual_result[1]
    assert expected_result[2] == actual_result[2]


def test_given_green_color_then_should_return_right_lower_boundary():
    expected_result = [47, 89, 0]

    actual_result = lower_boundary.get_lower_boundaries(green_color)

    assert expected_result[0] == actual_result[0]
    assert expected_result[1] == actual_result[1]
    assert expected_result[2] == actual_result[2]


def test_given_not_a_valid_color_then_should_return_array_of_zeros():
    expected_result = [0, 0, 0]

    actual_result = lower_boundary.get_lower_boundaries(not_a_color)

    assert expected_result[0] == actual_result[0]
    assert expected_result[1] == actual_result[1]
    assert expected_result[2] == actual_result[2]
