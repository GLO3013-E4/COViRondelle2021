import json

import rospy
from geometry_msgs.msg import PoseStamped
from handlers.handler import Handler
from std_msgs.msg import String


class MoveRobotHandler(Handler):
    def __init__(self):
        self.initialized = False
        self.is_finished = False

    def initialize(self):
        self.pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
        self.sub = rospy.Subscriber('movement_vectors_string', String, self.is_vector_at_destination)
        self.initialized = True

    def handle(self, handled_data=None):

        if not self.initialized:
            self.initialize()

        self.pub.publish(handled_data['goal'])

        while not self.is_finished:
            pass

        return handled_data

    def is_vector_at_destination(self, vector_json):
        vector = json.loads(str(vector_json.data))
        self.is_finished = vector == (0, 0, 0)

    def unregister(self):
        self.pub.unregister()
        self.sub.unregister()
