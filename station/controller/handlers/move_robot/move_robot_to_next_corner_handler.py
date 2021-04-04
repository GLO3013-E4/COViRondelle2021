import json
import rospy

from std_msgs.msg import String
from handlers.handler import Handler
from handlers.move_robot.move_robot_handler import MoveRobotHandler


class MoveRobotToNextCornerHandler(Handler):
    def handle(self, handled_data=None):
        move_robot_handler = MoveRobotHandler()
        current_puck = handled_data['current_puck']
        handled_data['goal'] = handled_data[current_puck]['corner_position']
        handled_data['destination'] = 'corner'

        is_finished = False
        while not is_finished:
            handled_data, is_finished = move_robot_handler.handle(handled_data)

        # TODO: avancer un peu pour coller le corner
        pub = rospy.Publisher('movement_vectors_string', String, queue_size=1)
        pub.publish(json.dumps("(5, 0, 0)"))

        return handled_data, True
