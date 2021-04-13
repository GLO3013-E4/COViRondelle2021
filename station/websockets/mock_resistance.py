import random

import rospy
from std_msgs.msg import String


def create_resistance():
    resistance = random.uniform(100, 1000000)
    return str(resistance)


# TODO : Remove this mock
if __name__ == '__main__':
    puck_colors_publisher = rospy.Publisher('resistance', String, queue_size=10)

    rospy.init_node('mock_resistance', anonymous=True)

    puck_colors_publisher.publish(create_resistance())
