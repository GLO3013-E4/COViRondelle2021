import json

import rospy
from handlers.handler import Handler
from handlers.move_robot.move_robot_handler import MoveRobotHandler
from std_msgs.msg import String
from utils import create_pose


class MoveRobotToNextCornerHandler(Handler):
    def __init__(self):
        self.initialized = False
        self.goal = None
        self.current_corner= None
        self.goal_tuple = None

    def initialize(self):
        self.pub = rospy.Publisher('movement_vectors_string', String, queue_size=1)
        self.sub = rospy.Subscriber('square', String, self.square_callback, queue_size=1)
        self.move_robot_handler = MoveRobotHandler()
        self.initialized = True

    def square_callback(self, data):
        square_dict = json.loads(data.data)
        self.goal_tuple = square_dict["corner_" + self.current_corner]
        self.sub.unregister()

    def handle(self, handled_data=None):
        if not self.initialized:
            self.initialize()

        self.current_corner = handled_data['corners'].pop()

        while self.goal_tuple is None or self.current_corner is None:
            pass

        handled_data["goal"] = create_pose(self.goal_tuple)

        handled_data = self.move_robot_handler.handle(handled_data)

        return handled_data

    def unregister(self):
        self.pub.unregister()
        self.sub.unregister()
        self.move_robot_handler.unregister()
