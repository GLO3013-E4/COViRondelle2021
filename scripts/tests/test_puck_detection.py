from scripts.src.detection.puck_detection import PuckDetection
color = "blue"
detection_puck = PuckDetection("images/monde2.jpg", color)


def test_given_an_area_in_range_then_return_true():
    area = 1700
    is_in_Area = detection_puck.is_in_area(area)
    assert True == is_in_Area


def test_given_an_area_not_in_range_then_return_false():
    area = 3000
    is_in_Area = detection_puck.is_in_area(area)
    assert False == is_in_Area


def test_given_coordinates_width_and_height_return_right_position():
    x = 20
    y = 120
    width = 47
    height = 50
    
    position_return = generate_puck_position(x, y, width, height)

    assert position_return["x_position"] == x
    assert position_return["y_position"] == y
    assert position_return["width"] == width
    assert position_return["height"] == height


def test_given_an_object_with_five_corner_then_return_name_puck():
    expectedName = f"{color} puck"
    
    actualName = detection_puck.get_object_name(5)

    assert expectedName == actualName


def test_given_an_object_with_three_corner_then_return_name():
    expectedName = "None"

    actualName = detection_puck.get_object_name(3)

    assert expectedName == actualName


def test_given_an_object_within_puck_dimension_then_should_be_in_range():
    width = 45
    height = 55

    object_is_in_range = detection_puck.object_is_in_range(width, height)

    assert True == object_is_in_range


def test_given_an_object_within_invalid_width_then_should_not_be_in_range():
    width = 33
    height = 55

    object_is_in_range = detection_puck.object_is_in_range(width, height)

    assert False == object_is_in_range


def test_given_an_object_within_invalid_height_then_should_not_be_in_range():
    width = 38
    height = 68

    object_is_in_range = detection_puck.object_is_in_range(width, height)

    assert False == object_is_in_range


def test_given_an_object_within_invalid_height_and_width_then_should_not_be_in_range():
    width = 28
    height = 68

    object_is_in_range = detection_puck.object_is_in_range(width, height)

    assert False == object_is_in_range


def test_given_an_image_with_blue_detection_required_then_return_right_positions():
    puck_detection = PuckDetection("monde.jpg", "yellow")

    position = puck_detection.detect_puck()

    assert True == isinstance(position, dict)




 
