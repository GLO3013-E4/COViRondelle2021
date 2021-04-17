import random

import rospy
from std_msgs.msg import String

rospy.init_node('mock_resistance', anonymous=True)


def create_resistance():
    resistance = random.randint(100, 1000000)
    return str(resistance)


def mock_resistance():
    puck_colors_publisher = rospy.Publisher('resistance', String, queue_size=10)

    puck_colors_publisher.publish(create_resistance())


if __name__ == '__main__':
    mock_resistance()
