import json
import rospy

from std_msgs.msg import String
from handlers.handler import Handler


class ReadLettersHandler(Handler):
    def handle(self, handled_data=None):

        self.is_finished = False
        rospy.Subscriber('letters', String, self.read_letters) # TODO: checker le nom du topic

        while not self.is_finished:
            pass

        handled_data['letters'] = self.letters

        return handled_data, True

    def read_letters(self, data):
        letters = json.loads(data.data)
        self.letters = letters
        self.is_finished = True
