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
        self.robot_angle = None
        self.juste_backed_up = False

    def initialize(self):
        self.sub = rospy.Subscriber("robot", String, self.callback, queue_size=1)
        self.initialized = True

    def callback(self, data):
        robot_dict = json.loads(data.data)
        self.position_tuple = robot_dict["prehenseur"]
        self.robot_angle = robot_dict["angle"]

    def handle(self, handled_data=None):
        self.initialize()
        handled_data["calculate_pucks_pub"].publish(True)
        while self.position_tuple is None:
            pass
        sauce = self.distance(self.position_tuple, (handled_data["goal"].pose.position.x, handled_data["goal"].pose.position.y))
        while self.distance(self.position_tuple, (handled_data["goal"].pose.position.x, handled_data["goal"].pose.position.y)) > 20:
            # if diff < -10 and not self.juste_backed_up:
            #     handled_data["movement_vectors_string_pub"].publish(json.dumps((30, 0 ,1)))
            #     self.juste_backed_up = True
            #     rospy.sleep(2)
            #     continue

            vector_angle = get_angle_between_two_points(handled_data["goal"].pose.position.x, handled_data["goal"].pose.position.y, *self.position_tuple)

            correction_angle = get_angle_correction(self.robot_angle, vector_angle)

            rospy.logerr(f"robot_angle : {math.degrees(self.robot_angle)} / {self.robot_angle}, vector_angle : {math.degrees(vector_angle)} / {vector_angle}, correction_angle : {math.degrees(correction_angle)} / {correction_angle}")

            if abs(correction_angle) > math.radians(5):
                correction_angle = 0

            handled_data["movement_vectors_string_pub"].publish(json.dumps((1, math.degrees(correction_angle), 0)))
            self.forwards_rate.sleep()
            handled_data["movement_vectors_string_pub"].publish(json.dumps((0, 0, 11)))
            self.forwards_rate.sleep()
            self.juste_backed_up = False

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


def get_angle_correction(robot_angle, vector_angle):
    if vector_angle < 0:
        vector_angle = 2 * math.pi + vector_angle
    if robot_angle < 0:
        robot_angle = 2 * math.pi + robot_angle

    angle_correction = vector_angle - robot_angle

    if angle_correction > math.pi:
        angle_correction -= 2 * math.pi
    elif angle_correction <= -math.pi:
        angle_correction += 2 * math.pi
    return angle_correction


def get_angle_between_two_points(x1, y1, x2, y2):
    angle = -math.atan2(y2 - y1, x2 - x1)

    if angle == -0:
        angle = 0
    elif angle == -math.pi:
        angle = math.pi
    return angle