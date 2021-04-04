import json

import rospy
from handlers.handler import Handler
from handlers.move_robot.move_robot_handler import MoveRobotHandler
from std_msgs.msg import String


class MoveRobotToNextPuckHandler(Handler):
    def __init__(self):
        self.initialized = False

    def initialize(self):
        self.pub = rospy.Publisher('movement_vectors_string', String, queue_size=1)
        self.move_robot_handler = MoveRobotHandler()
        self.initialized = True


    def handle(self, handled_data=None):
        if not self.initialized:
            self.initialize()

        if handled_data['current_puck'] == 'first_puck':
            handled_data['current_puck'] = 'second_puck'
        elif handled_data['current_puck'] == 'second_puck':
            handled_data['current_puck'] = 'third_puck'
        else:
            handled_data['current_puck'] = 'first_puck'

        current_puck = handled_data['current_puck']
        handled_data['goal'] = handled_data[current_puck]['position']
        handled_data['destination'] = 'puck'

        handled_data = move_robot_handler.handle(handled_data)

        #sleep ou on devrait enlever ça
        self.pub.publish(json.dumps("(5, 0, 0)"))

        return handled_data

    def unregister(self):
        self.pub.unregister()
        self.move_robot_handler.unregister()
