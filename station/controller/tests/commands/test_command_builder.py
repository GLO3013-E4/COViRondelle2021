from controller.src.commands.step import Step
from controller.src.commands.command_builder import CommandBuilder

from controller.src.handlers.wait_for_ready_state_handler import WaitForReadyStateHandler
from controller.src.handlers.send_ready_state_handler import SendReadyStateHandler
from controller.src.handlers.send_table_image.capture_table_image_handler import CaptureTableImageHandler
from controller.src.handlers.send_table_image.send_table_image_handler import SendTableImageHandler
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
from controller.src.handlers.get_next_corner_position_handler import GetNextCornerPositionHandler

command_builder = CommandBuilder()


def test_given_no_step_when_building_then_return_empty_list():
    steps = []

    commands = command_builder.with_steps(steps).build_many()

    assert len(commands) == 0


def test_given_multiple_steps_when_building_then_list_of_length_of_steps():
    steps = [Step.WaitForReadyState, Step.SendReadyState]

    commands = command_builder.with_steps(steps).build_many()

    assert len(commands) == len(steps)


def test_given_wait_for_ready_state_step_when_building_then_return_wait_for_ready_command():
    step = Step.WaitForReadyState
    handler_classes = [WaitForReadyStateHandler]

    given_single_step_when_building_then_return_correct_command(step, handler_classes)


def test_given_send_ready_state_step_when_building_then_return_send_ready_command():
    step = Step.SendReadyState
    handler_classes = [SendReadyStateHandler]

    given_single_step_when_building_then_return_correct_command(step, handler_classes)


def test_given_send_table_image_step_when_building_then_return_send_table_image_command():
    step = Step.SendTableImage
    handler_classes = [CaptureTableImageHandler, SendTableImageHandler]

    given_single_step_when_building_then_return_correct_command(step, handler_classes)


def test_given_get_resistance_station_position_step_when_building_then_return_get_resistance_station_position_command():
    step = Step.GetResistanceStationPosition
    handler_classes = [GetResistanceStationPositionHandler]

    given_single_step_when_building_then_return_correct_command(step, handler_classes)


def test_given_move_robot_step_when_building_then_return_move_robot_command():
    step = Step.MoveRobot
    handler_classes = [
        GetRobotPositionHandler,
        CalculateTrajectoryHandler,
        SendPlannedTrajectoryHandler,
        SendRealTrajectoryCoordinateHandler,
        WaitForRobotArrivalHandler
    ]

    given_single_step_when_building_then_return_correct_command(step, handler_classes)


def test_given_read_resistance_step_when_building_then_return_read_resistance_command():
    step = Step.ReadResistance
    handler_classes = [ReadResistanceHandler]

    given_single_step_when_building_then_return_correct_command(step, handler_classes)


def test_given_map_resistance_to_puck_colors_step_when_building_then_return_map_resistance_to_puck_colors_command():
    step = Step.MapResistanceToPuckColors
    handler_classes = [MapResistanceToPuckColorsHandler, SendResistanceAndPuckColorsHandler]

    given_single_step_when_building_then_return_correct_command(step, handler_classes)


def test_given_get_command_panel_station_position_step_when_building_then_return_get_command_panel_position_command():
    step = Step.GetCommandPanelPosition
    handler_classes = [GetCommandPanelPositionHandler]

    given_single_step_when_building_then_return_correct_command(step, handler_classes)


def test_given_read_letters_step_when_building_then_return_read_letters_command():
    step = Step.ReadLetters
    handler_classes = [ReadLettersHandler]

    given_single_step_when_building_then_return_correct_command(step, handler_classes)


def test_given_map_letters_to_puck_corners_step_when_building_then_return_map_letters_to_puck_corners_command():
    step = Step.MapLettersToPuckCorners
    handler_classes = [MapLettersToPuckCornersHandler, SendFirstPuckCornerHandler]

    given_single_step_when_building_then_return_correct_command(step, handler_classes)


def test_given_get_next_puck_position_step_when_building_then_return_get_next_puck_position_command():
    step = Step.GetNextPuckPosition
    handler_classes = [GetNextPuckPositionHandler]

    given_single_step_when_building_then_return_correct_command(step, handler_classes)


def test_given_get_next_corner_position_step_when_building_then_return_get_next_corner_position_command():
    step = Step.GetNextCornerPosition
    handler_classes = [GetNextCornerPositionHandler]

    given_single_step_when_building_then_return_correct_command(step, handler_classes)


def given_single_step_when_building_then_return_correct_command(step, handler_classes):
    commands = command_builder.with_steps([step]).build_many()

    assert len(commands) == 1
    assert len(commands[0].handlers) == len(handler_classes)

    for idx, handler_class in enumerate(handler_classes):
        assert isinstance(commands[0].handlers[idx], handler_class)
