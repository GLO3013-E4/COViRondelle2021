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


# TODO : Remove this mock
if __name__ == '__main__':
    puck_colors_publisher = rospy.Publisher('robot_consumption', String, queue_size=10)

    rospy.init_node('mock_robot_consumption', anonymous=True)

    puck_colors_publisher.publish(create_robot_consumption())
