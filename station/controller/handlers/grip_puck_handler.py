import json

import rospy

from handlers.handler import Handler


class GripPuckHandler(Handler):
    def __init__(self):
        self.initialized = False
        self.rate = rospy.Rate(0.5)
        self.GRAB = 7
        self.DROP = 8
        self.RAISE = 9
        self.LOWER = 10

    def handle(self, handled_data=None):

        self.rate.sleep()
        handled_data["movement_vectors_string_pub"].publish(json.dumps((0, 0 ,self.DROP)))
        self.rate.sleep()

        handled_data["movement_vectors_string_pub"].publish(json.dumps((15, 0 , 0)))
        self.rate.sleep()

        handled_data["movement_vectors_string_pub"].publish(json.dumps((0, 0 ,self.GRAB)))
        self.rate.sleep()

        handled_data["movement_vectors_string_pub"].publish(json.dumps((20, 0 , 1)))
        self.rate.sleep()
        return handled_data

    def unregister(self):
        pass
