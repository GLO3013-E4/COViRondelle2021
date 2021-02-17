from scripts.src.detection.qr_code_detection import QrDetection

INVALID_QR_CODE = "HelloWorld"
ROBOT_TYPE = "robot"
OBSTACLE_TYPE = "obstacle"
ROBOT_AND_OBSTACLE_TYPE = "both"
AN_IMAGE = "camera_monde2.jpg"
INVALID_IMAGE = "HelloWorld.jpg"

qr_detection = QrDetection(AN_IMAGE)

def test_given_invalid_qr_code_then_return_Zero():
    expected_position = qr_detection.detect_qr_code(INVALID_QR_CODE)

    assert expected_position == 0


def test_given_valid_image_when_detect_robot_should_return_dictionary_of_length_four():
    expected_position = qr_detection.detect_qr_code(ROBOT_TYPE)

    assert len(expected_position) == 4

def test_given_valid_image_when_detect_robot_should_return_dictionary_of_point():
    expected_position = qr_detection.detect_qr_code(ROBOT_TYPE)

    assert isinstance(expected_position["point 1"].x, int) is True
    assert isinstance(expected_position["point 1"].y, int) is True


def test_given_valid_image_when_detect_obstacle_should_return_array_of_length_two():
    expected_position = qr_detection.detect_qr_code(OBSTACLE_TYPE)

    assert len(expected_position) == 2

def test_given_valid_image_when_detect_obstacle_should_return_array_of_coordinate():
    expected_position = qr_detection.detect_qr_code(OBSTACLE_TYPE)

    assert isinstance(expected_position[0]["point 1"].x, int) is True
    assert isinstance(expected_position[0]["point 2"].x, int) is True

def test_given_valid_image_when_detect_robot_and_obstacle_should_return_dictionary_of_length_two():
    expected_position = qr_detection.detect_qr_code(ROBOT_AND_OBSTACLE_TYPE)

    assert len(expected_position) == 2
