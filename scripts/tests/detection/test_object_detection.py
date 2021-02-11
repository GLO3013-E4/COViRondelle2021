from scripts.src.detection.object_detection import ObjectDetection

A_IMAGE = "monde.jpg"
A_NAME = "Hello"
MINIMUM_DIMENSION = 30
MAXIMUM_DIMENSION = 70


object_detection = ObjectDetection(A_IMAGE, A_NAME, MINIMUM_DIMENSION, MAXIMUM_DIMENSION)


def test_given_coordinates_width_and_height_return_right_position():
    position_x = 20
    position_y = 120
    width = 47
    height = 50

    position_return = object_detection.generate_puck_position(position_x, position_y, width, height)

    assert position_return["x_position"] == position_x
    assert position_return["y_position"] == position_y
    assert position_return["width"] == width
    assert position_return["height"] == height

def test_given_an_object_within_puck_dimension_then_should_be_in_range():
    width = 45
    height = 55

    object_is_in_range = object_detection.object_is_in_range(width, height)

    assert object_is_in_range

def test_given_an_object_within_invalid_width_then_should_not_be_in_range():
    width = 23
    height = 55

    object_is_in_range = object_detection.object_is_in_range( width, height )

    assert not object_is_in_range


def test_given_an_object_within_invalid_height_then_should_not_be_in_range():
    width = 38
    height = 78

    object_is_in_range = object_detection.object_is_in_range( width, height )

    assert not object_is_in_range


def test_given_an_object_within_invalid_height_and_width_then_should_not_be_in_range():
    width = 28
    height = 68

    object_is_in_range = object_detection.object_is_in_range( width, height )

    assert not object_is_in_range