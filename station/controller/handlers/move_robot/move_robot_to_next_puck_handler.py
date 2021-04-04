from handlers.handler import Handler
from handlers.move_robot.move_robot_handler import MoveRobotHandler


class MoveRobotToNextPuckHandler(Handler):
    def handle(self, handled_data=None):
        if handled_data['current_puck'] == 'first_puck':
            handled_data['current_puck'] = 'second_puck'
        elif handled_data['current_puck'] == 'second_puck':
            handled_data['current_puck'] = 'third_puck'
        else:
            handled_data['current_puck'] = 'first_puck'

        move_robot_handler = MoveRobotHandler()
        current_puck = handled_data['current_puck']
        handled_data['goal'] = handled_data[current_puck]['position']

        is_finished = False
        while not is_finished:
            handled_data, is_finished = move_robot_handler.handle(handled_data)

        return handled_data, True
