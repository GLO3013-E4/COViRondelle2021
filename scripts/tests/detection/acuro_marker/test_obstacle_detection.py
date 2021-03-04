from scripts.src.detection.acuro_markers.obstacle_detection import ObstacleDetection

A_IMAGE = "robot_5x5_five.jpg"
A_INVALID_IMAGE = "invalid_image.jpg"

obstacle_detection = ObstacleDetection()

def test_given_a_valid_image_should_return_an_list():
    expected_position = obstacle_detection.detect_obstacle(A_IMAGE, DEBUG=False)

    assert isinstance(expected_position, list)

def test_given_a_valid_image_should_return_a_list_of_2_items():
    expected_position = obstacle_detection.detect_obstacle(A_IMAGE, DEBUG=False)

    assert len(expected_position) == 2

def test_given_a_valid_image_should_return_list_with_valid_fist_center():
    expected_position = obstacle_detection.detect_obstacle(A_IMAGE, DEBUG=False)

    assert expected_position[0]["obstacle 1"]["center"][0] >= 0
    assert expected_position[0]["obstacle 1"]["center"][1] >= 0

def test_given_a_valid_image_should_return_list_with_valid_second_center():
    expected_position = obstacle_detection.detect_obstacle(A_IMAGE, DEBUG=False)

    assert expected_position[1]["obstacle 2"]["center"][0] >= 0
    assert expected_position[1]["obstacle 2"]["center"][1] >= 0
