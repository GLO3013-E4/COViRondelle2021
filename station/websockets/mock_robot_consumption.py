import json
import random
import rospy
from std_msgs.msg import String


class MockRobotConsumption:
    def __init__(self):
        self.wheel1 = random.randint(0, 10)
        self.wheel2 = random.randint(0, 10)
        self.wheel3 = random.randint(0, 10)
        self.wheel4 = random.randint(0, 10)
        self.total = self.wheel1 + self.wheel2 + self.wheel3 + self.wheel4
        self.remainingTime = random.randint(0, 600)
        self.batteryCharge = random.uniform(0, 8)


def create_robot_consumption():
    robot_consumption = MockRobotConsumption()
    return json.dumps(robot_consumption)


# TODO : Remove this mock
if __name__ == '__main__':
    puck_colors_publisher = rospy.Publisher('robot_consumption', String, queue_size=10)

    rospy.init_node('mock_robot_consumption', anonymous=True)

    puck_colors_publisher.publish(create_robot_consumption())
