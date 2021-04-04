import json

import rospy
from handlers.handler import Handler
from handlers.move_robot.move_robot_handler import MoveRobotHandler
from std_msgs.msg import String


class MoveRobotToNextCornerHandler(Handler):
    def __init__(self):
        self.initialized = False

    def initialize(self):
        self.pub = rospy.Publisher('movement_vectors_string', String, queue_size=1)
        self.move_robot_handler = MoveRobotHandler()
        self.initialized = True

    def handle(self, handled_data=None):
        if not self.initialized:
            self.initialize()

        current_puck = handled_data['current_puck']
        handled_data['goal'] = handled_data[current_puck]['corner_position']
        handled_data['destination'] = 'corner'

        handled_data = self.move_robot_handler.handle(handled_data)

        self.pub.publish(json.dumps("(5, 0, 0)"))
        # sleep?

        return handled_data

    def unregister(self):
        self.pub.unregister()
        self.move_robot_handler.unregister()
