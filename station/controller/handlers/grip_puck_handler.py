import json

import rospy
from std_msgs.msg import String

from handlers.handler import Handler


class GripPuckHandler(Handler):
    def __init__(self):
        self.initialized = False
        self.rate = rospy.Rate(1)
        self.GRAB = 7
        self.RAISE = 9

    def initialize(self):
        self.pub = rospy.Publisher('movement_vectors_string', String, queue_size=1)
        self.initialized = True

    def handle(self, handled_data=None):
        if not self.initialized:
            self.initialize()

        self.pub(json.dumps((0, 0 ,self.GRAB)))
        self.rate.sleep()

        self.pub(json.dumps((0, 0, self.RAISE)))
        self.rate.sleep()
        return handled_data

    def unregister(self):
        self.pub.unregister()
