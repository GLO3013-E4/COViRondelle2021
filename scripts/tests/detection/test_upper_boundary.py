from scripts.src.detection.upper_boundary import UpperBoundary

upper_boundary = UpperBoundary()
red_color = "red"
blue_color = "blue"
yellow_color = "yellow"
not_a_color = "Hello world"


def test_given_red_color_then_should_return_right_upper_boundary():
    expected_result = [179, 255, 106]

    actual_result = upper_boundary.get_upper_boundaries(red_color)

    assert expected_result[0] == actual_result[0]
    assert expected_result[1] == actual_result[1]
    assert expected_result[2] == actual_result[2]


def test_given_blue_color_then_should_return_right_lower_boundary():
    expected_result = [179, 255, 255]

    actual_result = upper_boundary.get_upper_boundaries(blue_color)

    assert expected_result[0] == actual_result[0]
    assert expected_result[1] == actual_result[1]
    assert expected_result[2] == actual_result[2]


def test_given_yellow_color_then_should_return_right_lower_boundary():
    expected_result = [79, 255, 228]

    actual_result = upper_boundary.get_upper_boundaries(yellow_color)

    assert expected_result[0] == actual_result[0]
    assert expected_result[1] == actual_result[1]
    assert expected_result[2] == actual_result[2]


def test_given_not_a_valid_color_then_should_return_array_of_zeros():
    expected_result = [0, 0, 0]

    actual_result = upper_boundary.get_upper_boundaries(not_a_color)

    assert expected_result[0] == actual_result[0]
    assert expected_result[1] == actual_result[1]
    assert expected_result[2] == actual_result[2]