from detection.acuro_markers.robot_detection import RobotDetection


A_IMAGE = "./detection/tests/images/robot_5x5_five.jpg"
A_INVALID_IMAGE = "invalid_image.jpg"

robot_detection = RobotDetection()

def test_given_valid_image_should_return_dictionary_of_5_position():
    expected_position = robot_detection.detect_robot(A_IMAGE, DEBUG=False)

    assert len(expected_position) == 5

def test_given_valid_image_should_return_dictionary():
    expected_position = robot_detection.detect_robot(A_IMAGE, DEBUG=False)

    assert isinstance(expected_position, dict)

def test_given_valid_image_should_return_a_valid_center_x_position():
    expected_position = robot_detection.detect_robot(A_IMAGE, DEBUG=False)

    assert expected_position["center"][0] >= 0

def test_given_valid_image_should_return_a_valid_center_y_position():
    expected_position = robot_detection.detect_robot(A_IMAGE, DEBUG=False)

    assert expected_position["center"][1] >= 0