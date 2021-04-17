import json
import random
from enum import Enum
import rospy
from std_msgs.msg import String

rospy.init_node('mock_letters', anonymous=True)


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


def mock_letters():
    puck_colors_publisher = rospy.Publisher('letters', String, queue_size=10)

    puck_colors_publisher.publish(create_letters())


if __name__ == '__main__':
    mock_letters()
