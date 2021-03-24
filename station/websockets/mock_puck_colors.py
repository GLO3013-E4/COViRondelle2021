import random
from enum import Enum
import rospy
from std_msgs.msg import String


class Color(Enum):
    yellow = 0,
    brown = 1,
    red = 2,
    pink = 3,
    orange = 4,
    black = 5,
    white = 6,
    green = 7,
    blue = 8,
    purple = 9,
    grey = 10


def create_puck_colors():
    puck_colors = [
        random.choice(list(Color)).name,
        random.choice(list(Color)).name,
        random.choice(list(Color)).name
    ]
    return ','.join(puck_colors)


# TODO : Remove this mock
if __name__ == '__main__':
    puck_colors_publisher = rospy.Publisher('puck_colors', String, queue_size=10)

    rospy.init_node('mock_puck_colors', anonymous=True)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        puck_colors_publisher.publish(create_puck_colors())
        rate.sleep()
