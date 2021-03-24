import random
from enum import Enum
import rospy
from std_msgs.msg import String


class Corner(Enum):
    A = 0,
    B = 1,
    C = 2,
    D = 3


def create_puck_corners():
    puck_corners = [
        random.choice(list(Corner)).name,
        random.choice(list(Corner)).name,
        random.choice(list(Corner)).name
    ]
    return ','.join(puck_corners)


# TODO : Remove this mock
if __name__ == '__main__':
    puck_colors_publisher = rospy.Publisher('puck_corners', String, queue_size=10)

    rospy.init_node('mock_puck_corners', anonymous=True)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        puck_colors_publisher.publish(create_puck_corners())
        rate.sleep()
