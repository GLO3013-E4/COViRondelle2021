from scripts.src.detection.qr_code_detection import QrDetection

ROBOT_TYPE = "robot"
AN_IMAGE = "camera_monde_qr.jpg"
AN_INVALID_IMAGE = "image_without_square.jpg"
AN_OBSTACLE_POSITION = []
AN_OBSTACLE_WITH_ONE_POSITION = [(0, 0)]
AN_OBSTACLE_WITH_TWO_POSITION = [(0, 0), (0, 0)]

qr_detection = QrDetection(AN_IMAGE)
qr_detection_with_invalid_image = QrDetection(AN_INVALID_IMAGE)


def test_given_an_empty_obstacle_then_should_add_to_position_to_obstacle():
    obstacle_positions = qr_detection.\
        add_empty_position_if_not_all_obstacle_found(AN_OBSTACLE_POSITION)

    assert len(obstacle_positions) == 2


def test_given_an_obstacle_with_one_position_then_should_add_to_position_to_obstacle():
    obstacle_positions = qr_detection.\
        add_empty_position_if_not_all_obstacle_found(AN_OBSTACLE_WITH_ONE_POSITION)

    assert len(obstacle_positions) == 2


def test_given_an_obstacle_with_two_position_then_should_return_the_same_obstacles():
    obstacle_positions = qr_detection.\
        add_empty_position_if_not_all_obstacle_found(AN_OBSTACLE_WITH_TWO_POSITION)

    assert len(obstacle_positions) == 2


def test_given_an_image_then_should_return_two_obstacle_position():
    obstacle_positions = qr_detection.detect_obstacle()

    assert len(obstacle_positions) == 2


def test_given_an_invalid_image_then_should_return_two_obstacle_position():
    obstacle_positions = qr_detection_with_invalid_image.detect_obstacle()

    assert len(obstacle_positions) == 2


def test_given_a_image_then_should_return_one_robot_position_with_4_point():
    robot_position = qr_detection.detect_robot()

    assert len(robot_position) == 4