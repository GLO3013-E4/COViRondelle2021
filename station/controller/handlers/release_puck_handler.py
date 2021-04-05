import json

import rospy
from std_msgs.msg import String

from handlers.handler import Handler


class ReleasePuckHandler(Handler):
    def __init__(self):
        self.initialized = False
        self.rate = rospy.Rate(0.5)
        self.DROP = 8
        self.UP = 9
        self.LOWER = 10


    def handle(self, handled_data=None):
        self.rate.sleep()
        handled_data["movement_vectors_string_pub"].publish(json.dumps((10, 0 , 0)))
        self.rate.sleep()

        handled_data["movement_vectors_string_pub"].publish(json.dumps((0, 0 ,self.DROP)))
        self.rate.sleep()

        handled_data["movement_vectors_string_pub"].publish(json.dumps((0, 0 ,self.UP)))
        self.rate.sleep()

        handled_data["movement_vectors_string_pub"].publish(json.dumps((10, 0 , 1)))
        self.rate.sleep()

        return handled_data

    def unregister(self):
        pass
