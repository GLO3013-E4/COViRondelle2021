from scripts.src.detection.qr_code_detection import QrDetection

ROBOT_TYPE = "robot"
AN_IMAGE = "camera_monde_qr.jpg"
AN_OBSTACLE_POSITION = []
AN_OBSTACLE_WITH_ONE_POSITION = [(0, 0)]

qr_detection = QrDetection(AN_IMAGE)

def test_given_an_empty_obstacle_then_should_add_to_position_to_obstacle():
    obstacle_positions = qr_detection.\
        add_empty_position_if_not_all_obstacle_found(AN_OBSTACLE_POSITION)

    assert len(obstacle_positions) == 2

def test_given_an_obstacle_with_one_position_then_should_add_to_position_to_obstacle():
    obstacle_positions = qr_detection.\
        add_empty_position_if_not_all_obstacle_found(AN_OBSTACLE_WITH_ONE_POSITION)

    assert len(obstacle_positions) == 2
