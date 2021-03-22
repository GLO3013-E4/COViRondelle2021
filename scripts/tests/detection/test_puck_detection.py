from scripts.src.detection.puck_detection import PuckDetection

puck_detection = PuckDetection()
AN_IMAGE = "../../data/images/monde3.jpg"
puck_position = puck_detection.detect_pucks( AN_IMAGE, "red", False )

PURPLE_HSV = [130, 50, 100]
BLUE_HSV = [105, 70, 200]
GREEN_HSV = [60, 110, 40]
INVALID_HSV = [0, 2, 0]


def test_given_puck_should_return_dict_with_tuple_position_and_int_radius():
    assert isinstance(puck_position, dict) is True
    assert isinstance(puck_position["center_position"], tuple) is True


def test_given_puck_should_return_dict_of_len_2():
    assert len(puck_position) == 2


def test_given_hsv_color_of_purple_should_return_string_color_blue():
    expected_result = puck_detection.find_hsv_color(PURPLE_HSV)

    assert expected_result == "purple"


def test_given_hsv_color_of_blue_should_return_string_color_blue():
    expected_result = puck_detection.find_hsv_color(BLUE_HSV)

    assert expected_result == "blue"


def test_given_hsv_color_of_red_should_return_string_color_red():
    expected_result = puck_detection.find_hsv_color(GREEN_HSV)

    assert expected_result == "green"


def test_given_invalid_hsv_should_return_none():
    expected_result = puck_detection.find_hsv_color(INVALID_HSV)

    assert expected_result == 'None'
