from obstacle_detection import ObstacleDetection

AN_IMAGE = "robot_5x5_three.jpg"
ANOTHER_IMAGE = "robot_5x5_one.jpg"
FOURTH_IMAGE = "robot_5x5_four.jpg"
FIFTH_IMAGE = "robot_5x5_five.jpg"
IMAGE_TESTING = "testing.jpg"

obstacle_detection = ObstacleDetection()
obstacle_detection.detect_obstacle(IMAGE_TESTING)