from controller.src.commands.command import Command
from controller.src.commands.step import Step

from controller.src.handlers.wait_for_robot_ready_state_handler import WaitForRobotReadyStateHandler
from controller.src.handlers.wait_for_frontend_cycle_start_handler import WaitForFrontendCycleStartHandler
from controller.src.handlers.move_robot.move_robot_to_resistance_station_handler import MoveRobotToResistanceStationHandler
from controller.src.handlers.move_robot.move_robot_to_command_panel_handler import MoveRobotToCommandPanelHandler
from controller.src.handlers.move_robot.move_robot_to_next_puck_handler import MoveRobotToNextPuckHandler
from controller.src.handlers.move_robot.move_robot_to_next_corner_handler import MoveRobotToNextCornerHandler
from controller.src.handlers.move_robot.move_robot_to_square_center_handler import MoveRobotToSquareCenterHandler
from controller.src.handlers.move_robot.wait_for_robot_arrival_handler import WaitForRobotArrivalHandler
from controller.src.handlers.read_resistance_handler import ReadResistanceHandler
from controller.src.handlers.read_letters_handler import ReadLettersHandler
from controller.src.handlers.grip_puck_handler import GripPuckHandler
from controller.src.handlers.release_puck_handler import ReleasePuckHandler
from controller.src.handlers.end_cycle_handler import EndCycleHandler


class CommandBuilder:
    _commands = []

    def some_commands(self):
        self._commands = []
        return self

    def with_steps(self, steps):
        for step in steps:
            self._with_step(step)

        return self

    def _with_step(self, step):
        if step == Step.WAIT_FOR_ROBOT_READY_STATE:
            self._commands.append(Command([WaitForRobotReadyStateHandler()]))
        elif step == Step.WAIT_FOR_FRONTEND_CYCLE_START:
            self._commands.append(Command([WaitForFrontendCycleStartHandler()]))
        elif step == Step.MOVE_ROBOT_TO_RESISTANCE_STATION:
            self._commands.append(Command([MoveRobotToResistanceStationHandler(), WaitForRobotArrivalHandler()]))
        elif step == Step.READ_RESISTANCE:
            self._commands.append(Command([ReadResistanceHandler()]))
        elif step == Step.MOVE_ROBOT_TO_COMMAND_PANEL:
            self._commands.append(Command([MoveRobotToCommandPanelHandler(), WaitForRobotArrivalHandler()]))
        elif step == Step.READ_LETTERS:
            self._commands.append(Command([ReadLettersHandler()]))
        elif step == Step.MOVE_ROBOT_TO_NEXT_PUCK:
            self._commands.append(Command([MoveRobotToNextPuckHandler(), WaitForRobotArrivalHandler()]))
        elif step == Step.GRIP_PUCK:
            self._commands.append(Command([GripPuckHandler()]))
        elif step == Step.MOVE_ROBOT_TO_NEXT_CORNER:
            self._commands.append(Command([MoveRobotToNextCornerHandler(), WaitForRobotArrivalHandler()]))
        elif step == Step.RELEASE_PUCK:
            self._commands.append(Command([ReleasePuckHandler()]))
        elif step == Step.MOVE_ROBOT_TO_SQUARE_CENTER:
            self._commands.append(Command([MoveRobotToSquareCenterHandler(), WaitForRobotArrivalHandler()]))
        elif step == Step.END_CYCLE:
            self._commands.append(Command([EndCycleHandler()]))

    def build_many(self):
        return self._commands
