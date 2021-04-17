import json

import rospy
from std_msgs.msg import String


def create_puck_is_in_grip():
    movement_vectors_string = (0, 0, 7)
    return json.dumps(movement_vectors_string)


# TODO : Remove this mock
if __name__ == '__main__':
    puck_colors_publisher = rospy.Publisher('movement_vectors_string', String, queue_size=10)

    rospy.init_node('mock_puck_is_in_grip', anonymous=True)

    puck_colors_publisher.publish(create_puck_is_in_grip())
