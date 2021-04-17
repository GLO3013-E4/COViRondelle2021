import json
import random
import rospy
from std_msgs.msg import String


def create_robot_consumption():
    robot_consumption = {
        "wheel1": random.randint(0, 10),
        "wheel2": random.randint(0, 10),
        "wheel3": random.randint(0, 10),
        "wheel4": random.randint(0, 10),
        "total": random.randint(0, 40),
        "remainingTime": random.randint(0, 600),
        "batteryCharge": random.uniform(0, 8)
    }
    return json.dumps(robot_consumption)


def mock_robot_consumption():
    puck_colors_publisher = rospy.Publisher('robot_consumption', String, queue_size=10)

    rospy.loginfo('Mocking robot_consumption')
    puck_colors_publisher.publish(create_robot_consumption())


if __name__ == '__main__':
    mock_robot_consumption()
