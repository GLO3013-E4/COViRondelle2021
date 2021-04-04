import json
import rospy

from std_msgs.msg import String
from handlers.handler import Handler


class ReadResistanceHandler(Handler):
    def handle(self, handled_data=None):

        self.is_finished = False
        rospy.Subscriber('resistance', String, self.read_resistance) # TODO: checker le nom du topic

        while not self.is_finished:
            pass

        handled_data['resistance'] = self.resistance

        return handled_data, True

    def read_resistance(self, data):
        resistance = json.loads(data.data)
        self.resistance = resistance
        self.is_finished = True
