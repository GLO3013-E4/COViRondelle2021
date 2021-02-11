from scripts.src.detection.obstacle_detection import ObstacleDetection

obstacle_detection = ObstacleDetection("monde.jpg")


def test_given_nothing_should_return_two():
    result = obstacle_detection.detect_obstacle()

    assert len(result) == 2

