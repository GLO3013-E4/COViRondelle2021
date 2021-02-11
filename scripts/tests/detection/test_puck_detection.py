from scripts.src.detection.puck_detection import PuckDetection

A_COLOR = "blue"

detection_puck = PuckDetection("monde.jpg", A_COLOR)


def test_given_an_area_in_range_then_return_true():
    area = 1700
    is_in_area = detection_puck.is_in_area( area )
    assert is_in_area


def test_given_an_area_not_in_range_then_return_false():
    area = 3000
    is_in_area = detection_puck.is_in_area(area)
    assert not is_in_area


def test_given_an_object_with_five_corner_then_return_name_puck():
    expected_name = f"{A_COLOR} puck"

    actual_name = detection_puck.get_object_name(5)

    assert expected_name == actual_name


def test_given_an_object_with_three_corner_then_return_name():
    expected_name = "None"

    actual_name = detection_puck.get_object_name(3)

    assert expected_name == actual_name