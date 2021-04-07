import json
import rospy

from std_msgs.msg import String
from handlers.handler import Handler
from mapping.command_panel import CommandPanel

class ReadLettersHandler(Handler):
    def initialize(self):
        self.sub = rospy.Subscriber('letters', String, self.read_letters) # TODO: checker le nom du topic
        self.is_finished = False

    def handle(self, handled_data=None):
        self.initialize()
        command_panel = CommandPanel()
        command_panel.set_resistance(handled_data['resistance'])

        self.handled_data = handled_data
        handled_data["read_letters_pub"].publish(True)

        while not self.is_finished:
            pass
        
        handled_data["letters"] = self.letters
        command_panel.set_mapped_letters(self.letters)

        first_corner = command_panel.find_first_corner_letter()
        second_corner = first_corner.get_next_letter()
        third_corner = second_corner.get_next_letter()

        handled_data["corners"] = [first_corner.value, second_corner.value, third_corner.value]

        return handled_data

    def read_letters(self, data):
        letters = json.loads(data.data)
        self.letters = letters
        rospy.logerr("READ LETTERS " + str(self.letters))
        self.is_finished = len(letters) == 9
        if not self.is_finished:
            self.handled_data["read_letters_pub"].publish(True)

    def unregister(self):
        self.sub.unregister()