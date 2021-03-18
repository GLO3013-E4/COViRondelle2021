import cv2
from scripts.src.detection.acuro_markers.robot_detection import RobotDetection


A_IMAGE = "robot_5x5_five.jpg"
A_READ_IMAGE = cv2.imread(A_IMAGE)


robot_detection = RobotDetection()



def test_given_valid_image_should_return_dictionary_of_5_position():
    expected_position, _ = robot_detection.detect_aruco_marker_on_robot(A_READ_IMAGE, DEBUG=False )

    assert len(expected_position) == 5


def test_given_valid_image_should_return_dictionary():
    expected_position, _ = robot_detection.detect_aruco_marker_on_robot(A_READ_IMAGE, DEBUG=False )

    assert isinstance(expected_position, dict)


def test_given_valid_image_should_return_a_valid_center_x_position():
    expected_position, _ = robot_detection.detect_aruco_marker_on_robot(A_READ_IMAGE, DEBUG=False )

    assert expected_position["center"][0] >= 0


def test_given_valid_image_should_return_a_valid_center_y_position():
    expected_position, _ = robot_detection.detect_aruco_marker_on_robot(A_READ_IMAGE, DEBUG=False )

    assert expected_position["center"][1] >= 0
