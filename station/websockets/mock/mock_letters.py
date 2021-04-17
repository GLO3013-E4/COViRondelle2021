import json
import random
from enum import Enum
import rospy
from std_msgs.msg import String


class Corner(Enum):
    A = 'A'
    B = 'B'
    C = 'C'
    D = 'D'


def create_letters():
    letters = [
        random.choice(list(Corner)).name,
        random.choice(list(Corner)).name,
        random.choice(list(Corner)).name
    ]
    return json.dumps(letters)


# TODO : Remove this mock
if __name__ == '__main__':
    puck_colors_publisher = rospy.Publisher('letters', String, queue_size=10)

    rospy.init_node('mock_letters', anonymous=True)

    puck_colors_publisher.publish(create_letters())
