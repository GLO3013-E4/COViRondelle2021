from robot_detection import RobotDetection

AN_IMAGE = "robot_5x5_three.jpg"
ANOTHER_IMAGE = "robot_5x5_one.jpg"
FOURTH_IMAGE = "robot_5x5_four.jpg"
FIFTH_IMAGE = "robot_5x5_five.jpg"

robot_detection = RobotDetection()
robot_detection.detect_obstacle(FIFTH_IMAGE)