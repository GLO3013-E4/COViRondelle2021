import json
import rospy

from std_msgs.msg import String
from handlers.handler import Handler
from mapping.command_panel import CommandPanel


class DecodeResistanceAndLettersHandler(Handler):
    def handle(self, handled_data=None):
        self.pucks = None
        rospy.Subscriber('pucks', String, self.callback_pucks)
        command_panel = CommandPanel()

        while self.pucks is None:
            pass

        square_coordinates = handled_data['square']
        command_panel.set_coordinates(square_coordinates)

        resistance = handled_data['resistance']
        command_panel.set_resistance(resistance)

        letters = handled_data['letters']
        command_panel.set_mapped_letters(letters)

        is_finished = False
        while not is_finished:
            try:
                first_corner, second_corner, third_corner = command_panel.get_goals()
                is_finished = True
            except Exception as e:
                print(e)
                continue

        first_puck = self.pucks[first_corner.color.value].pop(0)["center_position"]
        handled_data['first_puck'] = {
            'position': first_puck,
            'color': first_corner.color.value,
            'corner_position': first_corner.position,
            'corner_letter': first_corner.letter.value #TODO: probablement pas necessaire mais peut-etre que ca peut aider pour le debug
        }

        second_puck = self.pucks[second_corner.color.value].pop(0)["center_position"]
        handled_data['second_puck'] = {
            'position': second_puck,
            'color': second_corner.color.value,
            'corner_position': second_corner.position,
            'corner_letter': second_corner.letter.value #TODO: probablement pas necessaire mais peut-etre que ca peut aider pour le debug
        }

        third_puck = self.pucks[third_corner.color.value].pop(0)["center_position"]
        handled_data['third_puck'] = {
            'position': third_puck,
            'color': third_corner.color.value,
            'corner_position': third_corner.position,
            'corner_letter': third_corner.letter.value #TODO: probablement pas necessaire mais peut-etre que ca peut aider pour le debug
        }

        return handled_data, True

    def callback_pucks(self, pucks):
        self.pucks = json.loads(pucks.data)
