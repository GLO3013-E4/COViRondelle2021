from scripts.src.detection.puck_detection import PuckDetection
from scripts.src.mapping.color import Color

A_COLOR = Color["BLUE"]

detection_puck = PuckDetection( "images/monde3.jpg", A_COLOR)


def test_given_an_area_in_range_then_return_true():
    area = 1700
    is_in_area = detection_puck.is_in_area(area)
    assert is_in_area


def test_given_an_area_not_in_range_then_return_false():
    area = 3000
    is_in_area = detection_puck.is_in_area(area)
    assert not is_in_area


def test_given_coordinates_width_and_height_return_right_position():
    position_x = 20
    position_y = 120
    width = 47
    height = 50
    position_return = detection_puck.generate_puck_position(position_x, position_y, width, height)

    assert position_return["x_position"] == position_x
    assert position_return["y_position"] == position_y
    assert position_return["width"] == width
    assert position_return["height"] == height


def test_given_an_object_with_five_corner_then_return_name_puck():
    expected_name = f"{A_COLOR} puck"

    actual_name = detection_puck.get_object_name(5)

    assert expected_name == actual_name


def test_given_an_object_with_three_corner_then_return_name():
    expected_name = "None"

    actual_name = detection_puck.get_object_name(3)

    assert expected_name == actual_name


def test_given_an_object_within_puck_dimension_then_should_be_in_range():
    width = 45
    height = 55

    object_is_in_range = detection_puck.object_is_in_range(width, height)

    assert object_is_in_range is True


def test_given_an_object_within_invalid_width_then_should_not_be_in_range():
    width = 33
    height = 55

    object_is_in_range = detection_puck.object_is_in_range(width, height)

    assert object_is_in_range is False


def test_given_an_object_within_invalid_height_then_should_not_be_in_range():
    width = 38
    height = 68

    object_is_in_range = detection_puck.object_is_in_range(width, height)

    assert object_is_in_range is False


def test_given_an_object_within_invalid_height_and_width_then_should_not_be_in_range():
    width = 28
    height = 68

    object_is_in_range = detection_puck.object_is_in_range(width, height)

    assert object_is_in_range is False
