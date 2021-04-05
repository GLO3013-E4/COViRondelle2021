from handlers.handler import Handler
from handlers.move_robot.move_robot_handler import MoveRobotHandler
from hard_coded_positions import COMMAND_PANEL_POSITION


class MoveRobotToCommandPanelHandler(Handler):
    def handle(self, handled_data=None):
        move_robot_handler = MoveRobotHandler()

        handled_data['goal'] = COMMAND_PANEL_POSITION
        handled_data['destination'] = 'command_panel'

        is_finished = False
        while not is_finished:
            handled_data, is_finished = move_robot_handler.handle(handled_data)

        return handled_data, True
