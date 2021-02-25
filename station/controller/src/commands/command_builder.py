from controller.src.commands.command import Command
from controller.src.commands.step import Step

from controller.src.handlers.wait_for_robot_ready_state_handler import WaitForRobotReadyStateHandler
from controller.src.handlers.send_to_frontend_ready_state_handler import SendToFrontendReadyStateHandler
from controller.src.handlers.wait_for_frontend_cycle_start_handler import WaitForFrontendCycleStartHandler
from controller.src.handlers.send_table_image.capture_table_image_handler import CaptureTableImageHandler
from controller.src.handlers.send_table_image.send_to_frontend_table_image_handler import SendToFrontendTableImageHandler
from controller.src.handlers.get_resistance_station_position_handler import GetResistanceStationPositionHandler
from controller.src.handlers.move_robot.get_robot_position_handler import GetRobotPositionHandler
from controller.src.handlers.move_robot.calculate_trajectory_handler import CalculateTrajectoryHandler
from controller.src.handlers.move_robot.send_to_robot_planned_trajectory_handler import SendToRobotPlannedTrajectoryHandler
from controller.src.handlers.move_robot.send_to_frontend_planned_trajectory_handler import SendToFrontendPlannedTrajectoryHandler
from controller.src.handlers.move_robot.send_real_trajectory_coordinate_handler import SendRealTrajectoryCoordinateHandler
from controller.src.handlers.move_robot.wait_for_robot_arrival_handler import WaitForRobotArrivalHandler
from controller.src.handlers.read_resistance_handler import ReadResistanceHandler
from controller.src.handlers.map_resistance_to_puck_colors.map_resistance_to_puck_colors_handler import MapResistanceToPuckColorsHandler
from controller.src.handlers.map_resistance_to_puck_colors.send_resistance_and_puck_colors_handler import SendResistanceAndPuckColorsHandler
from controller.src.handlers.get_command_panel_position_handler import GetCommandPanelPositionHandler
from controller.src.handlers.read_letters_handler import ReadLettersHandler
from controller.src.handlers.map_letters_to_puck_corners.map_letters_to_puck_corners_handler import MapLettersToPuckCornersHandler
from controller.src.handlers.map_letters_to_puck_corners.send_first_puck_corner_handler import SendFirstPuckCornerHandler
from controller.src.handlers.get_next_puck_position_handler import GetNextPuckPositionHandler
from controller.src.handlers.grip_puck.grip_puck_handler import GripPuckHandler
from controller.src.handlers.grip_puck.send_puck_gripped_state_handler import SendPuckGrippedStateHandler
from controller.src.handlers.get_next_corner_position_handler import GetNextCornerPositionHandler
from controller.src.handlers.release_puck.release_puck_handler import ReleasePuckHandler
from controller.src.handlers.release_puck.send_puck_released_state_handler import SendPuckReleasedStateHandler
from controller.src.handlers.get_start_square_center_position_handler import GetStartSquareCenterPositionHandler
from controller.src.handlers.end_cycle.turn_on_red_light_handler import TurnOnRedLightHandler
from controller.src.handlers.end_cycle.send_cycle_ended_step_handler import SendCycleEndedStepHandler


class CommandBuilder:
    _commands = []

    def with_steps(self, steps):
        self._commands = []

        for step in steps:
            self._with_step(step)

        return self

    def _with_step(self, step):
        if step == Step.WAIT_FOR_ROBOT_READY_STATE:
            self._commands.append(Command([WaitForRobotReadyStateHandler()]))
        elif step == Step.SEND_TO_FRONTEND_READY_STATE:
            self._commands.append(Command([SendToFrontendReadyStateHandler()]))
        elif step == Step.WAIT_FOR_FRONTEND_CYCLE_START:
            self._commands.append(Command([WaitForFrontendCycleStartHandler()]))
        elif step == Step.SEND_TABLE_IMAGE:
            self._commands.append(Command([CaptureTableImageHandler(), SendToFrontendTableImageHandler()]))
        elif step == Step.GET_RESISTANCE_STATION_POSITION:
            self._commands.append(Command([GetResistanceStationPositionHandler()]))
        elif step == Step.MOVE_ROBOT:
            self._commands.append(Command([
                GetRobotPositionHandler(),
                CalculateTrajectoryHandler(),
                SendToRobotPlannedTrajectoryHandler(),
                SendToFrontendPlannedTrajectoryHandler(),
                SendRealTrajectoryCoordinateHandler(),
                WaitForRobotArrivalHandler()
            ]))
        elif step == Step.READ_RESISTANCE:
            self._commands.append(Command([ReadResistanceHandler()]))
        elif step == Step.MAP_RESISTANCE_TO_PUCK_COLORS:
            self._commands.append(Command([MapResistanceToPuckColorsHandler(), SendResistanceAndPuckColorsHandler()]))
        elif step == Step.GET_COMMAND_PANEL_POSITION:
            self._commands.append(Command([GetCommandPanelPositionHandler()]))
        elif step == Step.READ_LETTERS:
            self._commands.append(Command([ReadLettersHandler()]))
        elif step == Step.MAP_LETTERS_TO_PUCK_CORNERS:
            self._commands.append(Command([MapLettersToPuckCornersHandler(), SendFirstPuckCornerHandler()]))
        elif step == Step.GET_NEXT_PUCK_POSITION:
            self._commands.append(Command([GetNextPuckPositionHandler()]))
        elif step == Step.GRIP_PUCK:
            self._commands.append(Command([GripPuckHandler(), SendPuckGrippedStateHandler()]))
        elif step == Step.GET_NEXT_CORNER_POSITION:
            self._commands.append(Command([GetNextCornerPositionHandler()]))
        elif step == Step.RELEASE_PUCK:
            self._commands.append(Command([ReleasePuckHandler(), SendPuckReleasedStateHandler()]))
        elif step == Step.GET_START_SQUARE_CENTER_POSITION:
            self._commands.append(Command([GetStartSquareCenterPositionHandler()]))
        elif step == Step.END_CYCLE:
            self._commands.append(Command([TurnOnRedLightHandler(), SendCycleEndedStepHandler()]))

    def build_many(self):
        return self._commands
