from scripts.src.detection.obstacle_detection import ObstacleDetection

obstacle_detection = ObstacleDetection("monde.jpg")


def test_given_nothing_should_not_be_empty():
    position_result = obstacle_detection.detect_obstacle()

    assert len(position_result) != 0

def test_detect_an_obstacle_then_should_return_two_position():
    position_result = obstacle_detection.detect_obstacle()

    assert len(position_result) == 2


