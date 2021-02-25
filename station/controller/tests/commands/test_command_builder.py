from controller.src.commands.step import Step
from controller.src.commands.command_builder import CommandBuilder

from controller.src.handlers.wait_for_robot_ready_state_handler import WaitForRobotReadyStateHandler
from controller.src.handlers.send_to_frontend_ready_state_handler import SendToFrontendReadyStateHandler
from controller.src.handlers.wait_for_frontend_cycle_start_handler import WaitForFrontendCycleStartHandler
from controller.src.handlers.send_table_image.capture_table_image_handler import CaptureTableImageHandler
from controller.src.handlers.send_table_image.send_to_frontend_table_image_handler import SendToFrontendTableImageHandler
from controller.src.handlers.get_resistance_station_position_handler import GetResistanceStationPositionHandler
from controller.src.handlers.move_robot.get_robot_position_handler import GetRobotPositionHandler
from controller.src.handlers.move_robot.calculate_trajectory_handler import CalculateTrajectoryHandler
from controller.src.handlers.move_robot.send_planned_trajectory_handler import SendPlannedTrajectoryHandler
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

command_builder = CommandBuilder()


def test_given_no_step_when_building_then_return_empty_list():
    steps = []

    commands = command_builder.with_steps(steps).build_many()

    assert len(commands) == 0


def test_given_multiple_steps_when_building_then_list_of_length_of_steps():
    steps = [Step.WAIT_FOR_ROBOT_READY_STATE, Step.SEND_TO_FRONTEND_READY_STATE]

    commands = command_builder.with_steps(steps).build_many()

    assert len(commands) == len(steps)


def test_given_wait_for_robot_ready_state_step_when_building_then_return_wait_for_robot_ready_command():
    step = Step.WAIT_FOR_ROBOT_READY_STATE
    handler_classes = [WaitForRobotReadyStateHandler]

    given_single_step_when_building_then_return_correct_command(step, handler_classes)


def test_given_send_to_frontend_ready_state_step_when_building_then_return_send_to_frontend_ready_command():
    step = Step.SEND_TO_FRONTEND_READY_STATE
    handler_classes = [SendToFrontendReadyStateHandler]

    given_single_step_when_building_then_return_correct_command(step, handler_classes)


def test_given_wait_for_frontend_cycle_start_step_when_building_then_return_wait_for_frontend_cycle_start_command():
    step = Step.WAIT_FOR_FRONTEND_CYCLE_START
    handler_classes = [WaitForFrontendCycleStartHandler]

    given_single_step_when_building_then_return_correct_command(step, handler_classes)


def test_given_send_table_image_step_when_building_then_return_send_table_image_command():
    step = Step.SEND_TABLE_IMAGE
    handler_classes = [CaptureTableImageHandler, SendToFrontendTableImageHandler]

    given_single_step_when_building_then_return_correct_command(step, handler_classes)


def test_given_get_resistance_station_position_step_when_building_then_return_get_resistance_station_position_command():
    step = Step.GET_RESISTANCE_STATION_POSITION
    handler_classes = [GetResistanceStationPositionHandler]

    given_single_step_when_building_then_return_correct_command(step, handler_classes)


def test_given_move_robot_step_when_building_then_return_move_robot_command():
    step = Step.MOVE_ROBOT
    handler_classes = [
        GetRobotPositionHandler,
        CalculateTrajectoryHandler,
        SendPlannedTrajectoryHandler,
        SendRealTrajectoryCoordinateHandler,
        WaitForRobotArrivalHandler
    ]

    given_single_step_when_building_then_return_correct_command(step, handler_classes)


def test_given_read_resistance_step_when_building_then_return_read_resistance_command():
    step = Step.READ_RESISTANCE
    handler_classes = [ReadResistanceHandler]

    given_single_step_when_building_then_return_correct_command(step, handler_classes)


def test_given_map_resistance_to_puck_colors_step_when_building_then_return_map_resistance_to_puck_colors_command():
    step = Step.MAP_RESISTANCE_TO_PUCK_COLORS
    handler_classes = [MapResistanceToPuckColorsHandler, SendResistanceAndPuckColorsHandler]

    given_single_step_when_building_then_return_correct_command(step, handler_classes)


def test_given_get_command_panel_station_position_step_when_building_then_return_get_command_panel_position_command():
    step = Step.GET_COMMAND_PANEL_POSITION
    handler_classes = [GetCommandPanelPositionHandler]

    given_single_step_when_building_then_return_correct_command(step, handler_classes)


def test_given_read_letters_step_when_building_then_return_read_letters_command():
    step = Step.READ_LETTERS
    handler_classes = [ReadLettersHandler]

    given_single_step_when_building_then_return_correct_command(step, handler_classes)


def test_given_map_letters_to_puck_corners_step_when_building_then_return_map_letters_to_puck_corners_command():
    step = Step.MAP_LETTERS_TO_PUCK_CORNERS
    handler_classes = [MapLettersToPuckCornersHandler, SendFirstPuckCornerHandler]

    given_single_step_when_building_then_return_correct_command(step, handler_classes)


def test_given_get_next_puck_position_step_when_building_then_return_get_next_puck_position_command():
    step = Step.GET_NEXT_PUCK_POSITION
    handler_classes = [GetNextPuckPositionHandler]

    given_single_step_when_building_then_return_correct_command(step, handler_classes)


def test_given_grip_puck_step_when_building_then_return_grip_puck_command():
    step = Step.GRIP_PUCK
    handler_classes = [GripPuckHandler, SendPuckGrippedStateHandler]

    given_single_step_when_building_then_return_correct_command(step, handler_classes)


def test_given_get_next_corner_position_step_when_building_then_return_get_next_corner_position_command():
    step = Step.GET_NEXT_CORNER_POSITION
    handler_classes = [GetNextCornerPositionHandler]

    given_single_step_when_building_then_return_correct_command(step, handler_classes)


def test_given_release_puck_step_when_building_then_return_release_puck_command():
    step = Step.RELEASE_PUCK
    handler_classes = [ReleasePuckHandler, SendPuckReleasedStateHandler]

    given_single_step_when_building_then_return_correct_command(step, handler_classes)


def test_given_get_start_square_center_position_step_when_building_then_return_get_start_square_center_position_command():
    step = Step.GET_START_SQUARE_CENTER_POSITION
    handler_classes = [GetStartSquareCenterPositionHandler]

    given_single_step_when_building_then_return_correct_command(step, handler_classes)


def test_given_end_cycle_step_when_building_then_return_end_cycle_command():
    step = Step.END_CYCLE
    handler_classes = [TurnOnRedLightHandler, SendCycleEndedStepHandler]

    given_single_step_when_building_then_return_correct_command(step, handler_classes)


def given_single_step_when_building_then_return_correct_command(step, handler_classes):
    commands = command_builder.with_steps([step]).build_many()

    assert len(commands) == 1
    assert len(commands[0].handlers) == len(handler_classes)

    for idx, handler_class in enumerate(handler_classes):
        assert isinstance(commands[0].handlers[idx], handler_class)
