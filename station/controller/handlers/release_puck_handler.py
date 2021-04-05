import json
import math
import rospy
from std_msgs.msg import String

from handlers.handler import Handler


class ReleasePuckHandler(Handler):
    def __init__(self):
        self.initialized = False
        self.rate = rospy.Rate(0.5)
        self.forwards_rate = rospy.Rate(1)
        self.DROP = 8
        self.UP = 9
        self.LOWER = 10
        self.BRAKES = 11
        self.position_tuple = None

    def initialize(self):
        self.sub = rospy.Subscriber("robot", String, self.callback, queue_size=1)
        self.initialized = True
    
    def callback(self, data):
        robot_dict = json.loads(data.data)
        self.position_tuple = robot_dict["prehenseur"]


    def handle(self, handled_data=None):
        self.initialize()
        while self.position_tuple is None:
            pass
        while self.distance(self.position_tuple, (handled_data["goal"].pose.position.x, handled_data["goal"].pose.position.y)) > 20:
            sauce = self.distance(self.position_tuple, (handled_data["goal"].pose.position.x, handled_data["goal"].pose.position.y))
            rospy.logerr(sauce)
            handled_data["movement_vectors_string_pub"].publish(json.dumps((1, 0 , 0)))
            self.forwards_rate.sleep()

        self.rate.sleep()
        handled_data["movement_vectors_string_pub"].publish(json.dumps((0, 0 ,self.DROP)))
        self.rate.sleep()

        handled_data["movement_vectors_string_pub"].publish(json.dumps((0, 0 ,self.UP)))
        self.rate.sleep()

        handled_data["movement_vectors_string_pub"].publish(json.dumps((10, 0 , 1)))
        self.rate.sleep()

        return handled_data

    def distance(self, point1, point2):
        x1, y1 = point1
        x2, y2 = point2
        return math.sqrt(pow(x2-x1, 2) + pow(y2-y1, 2))

    def unregister(self):
        self.sub.unregister()
