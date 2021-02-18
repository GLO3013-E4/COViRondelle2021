import pytest
from scripts.src.detection.qr_code_detection import QrDetection

INVALID_QR_CODE = "HelloWorld"
ROBOT_TYPE = "robot"
OBSTACLE_TYPE = "obstacle"
ROBOT_AND_OBSTACLE_TYPE = "both"
AN_IMAGE = "camera_monde_qr.jpg"
INVALID_IMAGE = "HelloWorld.jpg"

qr_detection = QrDetection(AN_IMAGE)
qr_detection_with_invalid_image = QrDetection(INVALID_IMAGE)


def test_given_invalid_qr_code_then_return_zero():
    expected_position = qr_detection.detect_qr_code(INVALID_QR_CODE)

    assert expected_position == 0


def test_given_valid_image_when_detect_robot_should_return_dictionary_of_number_wit_x_position():
    expected_position = qr_detection.detect_qr_code(ROBOT_TYPE)

    assert isinstance(expected_position["point 1"].x, int) is True
    assert isinstance(expected_position["point 2"].x, int) is True
    assert isinstance(expected_position["point 3"].x, int) is True
    assert isinstance(expected_position["point 4"].x, int) is True


def test_given_valid_image_when_detect_robot_should_return_dictionary_of_number_wit_y_position():
    expected_position = qr_detection.detect_qr_code(ROBOT_TYPE)

    assert isinstance(expected_position["point 1"].y, int) is True
    assert isinstance(expected_position["point 2"].y, int) is True
    assert isinstance(expected_position["point 3"].y, int) is True
    assert isinstance(expected_position["point 4"].y, int) is True


def test_given_valid_image_when_detect_robot_should_return_dictionary_of_x_positive_number():
    expected_position = qr_detection.detect_qr_code(ROBOT_TYPE)

    assert expected_position["point 1"].x > 0
    assert expected_position["point 2"].x > 0
    assert expected_position["point 2"].x > 0
    assert expected_position["point 2"].x > 0


def test_given_valid_image_when_detect_robot_should_return_dictionary_of_y_positive_number():
    expected_position = qr_detection.detect_qr_code(ROBOT_TYPE)

    assert expected_position["point 1"].y > 0
    assert expected_position["point 2"].y > 0
    assert expected_position["point 2"].y > 0
    assert expected_position["point 2"].y > 0


def test_given_valid_image_when_detect_obstacle_should_return_array_of_length_two():
    expected_position = qr_detection.detect_qr_code(OBSTACLE_TYPE)

    assert len(expected_position) == 2


def test_given_valid_image_when_detect_obstacle_should_return_array_of_x_coordinate_number():
    expected_position = qr_detection.detect_qr_code(OBSTACLE_TYPE)

    assert isinstance(expected_position[0]["point 1"].x, int) is True
    assert isinstance(expected_position[0]["point 2"].x, int) is True
    assert isinstance(expected_position[0]["point 3"].x, int) is True
    assert isinstance(expected_position[0]["point 4"].x, int) is True


def test_given_valid_image_when_detect_obstacle_should_return_array_of_y_coordinate_number():
    expected_position = qr_detection.detect_qr_code(OBSTACLE_TYPE)

    assert isinstance(expected_position[0]["point 1"].y, int) is True
    assert isinstance(expected_position[0]["point 2"].y, int) is True
    assert isinstance(expected_position[0]["point 3"].y, int) is True
    assert isinstance(expected_position[0]["point 4"].y, int) is True


def test_given_valid_image_detect_robot_obstacle_should_return_dictionary_of_length_two():
    expected_position = qr_detection.detect_qr_code(ROBOT_AND_OBSTACLE_TYPE)

    assert len(expected_position) == 2


def test_given_valid_image_detect_robot_obstacle_should_return_dictionary_with_one_robot_position():
    expected_position = qr_detection.detect_qr_code(ROBOT_AND_OBSTACLE_TYPE)

    assert len(expected_position["robot"]) == 1


def test_given_valid_image_detect_robot_obstacle_should_return_dictionary_with_two_obstacle():
    expected_position = qr_detection.detect_qr_code(ROBOT_AND_OBSTACLE_TYPE)

    assert len(expected_position["obstacles"]) == 2
