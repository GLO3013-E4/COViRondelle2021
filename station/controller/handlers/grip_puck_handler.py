import json

import rospy
from std_msgs.msg import String

from handlers.handler import Handler


class GripPuckHandler(Handler):
    def __init__(self):
        self.initialized = False
        self.rate = rospy.Rate(0.5)
        self.GRAB = 7
        self.RAISE = 9

    def handle(self, handled_data=None):

        handled_data["movement_vectors_string_pub"].publish(json.dumps((0, 0 ,self.GRAB)))
        self.rate.sleep()

        handled_data["movement_vectors_string_pub"].publish(json.dumps((0, 0 ,self.GRAB)))
        self.rate.sleep()

        handled_data["movement_vectors_string_pub"].publish(json.dumps((0, 0, self.RAISE)))
        self.rate.sleep()
        return handled_data

    def unregister(self):
        pass
