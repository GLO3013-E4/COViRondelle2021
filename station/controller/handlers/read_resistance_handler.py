import json
import rospy

from std_msgs.msg import String
from handlers.handler import Handler
from mapping.resistance import Resistance
from mapping.resistance_mapper import ResistanceMapper


class ReadResistanceHandler(Handler):
    def initialize(self):
        self.sub = rospy.Subscriber('resistance', String, self.read_resistance) # TODO: checker le nom du topic
        self.is_finished = False

    def handle(self, handled_data=None):
        self.initialize()

        while not self.is_finished:
            pass

        handled_data['resistance'] = self.resistance
        
        handled_data["puck_colors"] = ResistanceMapper().find_colors(Resistance(self.resistance))

        return handled_data

    def read_resistance(self, data):
        resistance = json.loads(data.data)
        self.resistance = resistance
        rospy.logerr("READ RESISTANCE " + self.resistance)
        self.is_finished = resistance != 0

    def unregister(self):
        self.sub.unregister()
