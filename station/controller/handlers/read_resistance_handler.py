import json
import rospy

from std_msgs.msg import String
from handlers.handler import Handler
from mapping.resistance import Resistance
from mapping.resistance_mapper import ResistanceMapper


class ReadResistanceHandler(Handler):
    def initialize(self):
        self.sub = rospy.Subscriber('resistance', String, self.read_resistance) # TODO: checker le nom du topic
        self.rate = rospy.Rate(0.5)
        self.is_finished = False

    def handle(self, handled_data=None):
        self.initialize()

        while not self.is_finished:
            handled_data["movement_vectors_string_pub"].publish(json.dumps((0, 0, 6)))
            rospy.logerr("grrrrrrrrrrrrrr")
            self.rate.sleep()
            pass

        handled_data['resistance'] = self.resistance
        
        handled_data["puck_colors"] = ResistanceMapper().find_colors(Resistance(self.resistance))

        self.rate.sleep()
        handled_data["movement_vectors_string_pub"].publish(json.dumps((handled_data["resistance_y_dist"], 0, 3)))
        self.rate.sleep()
        handled_data["movement_vectors_string_pub"].publish(json.dumps((handled_data["resistance_x_dist"], 0, 0)))
        self.rate.sleep()

        return handled_data

    def read_resistance(self, data):
        resistance = json.loads(data.data)
        self.resistance = resistance
        rospy.logerr("READ RESISTANCE " + str(self.resistance))
        self.is_finished = resistance != 0

    def unregister(self):
        self.sub.unregister()
