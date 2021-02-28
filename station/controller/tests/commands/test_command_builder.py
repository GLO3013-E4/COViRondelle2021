from controller.src.commands.step import Step
from controller.src.commands.command_builder import CommandBuilder

from controller.src.handlers.wait_for_robot_ready_state_handler import WaitForRobotReadyStateHandler
from controller.src.handlers.wait_for_frontend_cycle_start_handler import WaitForFrontendCycleStartHandler
from controller.src.handlers.move_robot.move_robot_to_resistance_station_handler import MoveRobotToResistanceStationHandler
from controller.src.handlers.move_robot.move_robot_to_command_panel_handler import MoveRobotToCommandPanelHandler
from controller.src.handlers.move_robot.move_robot_to_next_puck_handler import MoveRobotToNextPuckHandler
from controller.src.handlers.move_robot.get_robot_position_handler import GetRobotPositionHandler
from controller.src.handlers.move_robot.calculate_trajectory_handler import CalculateTrajectoryHandler
from controller.src.handlers.move_robot.send_to_robot_planned_trajectory_handler import SendToRobotPlannedTrajectoryHandler
from controller.src.handlers.move_robot.send_to_frontend_planned_trajectory_handler import SendToFrontendPlannedTrajectoryHandler
from controller.src.handlers.move_robot.send_to_frontend_real_trajectory_coordinate_handler import SendToFrontendRealTrajectoryCoordinateHandler
from controller.src.handlers.move_robot.wait_for_robot_arrival_handler import WaitForRobotArrivalHandler
from controller.src.handlers.read_resistance_handler import ReadResistanceHandler
from controller.src.handlers.read_letters_handler import ReadLettersHandler
from controller.src.handlers.grip_puck_handler import GripPuckHandler
from controller.src.handlers.get_next_corner_position_handler import GetNextCornerPositionHandler
from controller.src.handlers.release_puck.release_puck_handler import ReleasePuckHandler
from controller.src.handlers.release_puck.send_to_frontend_puck_released_state_handler import SendToFrontendPuckReleasedStateHandler
from controller.src.handlers.get_start_square_center_position_handler import GetStartSquareCenterPositionHandler
from controller.src.handlers.end_cycle.turn_on_red_light_handler import TurnOnRedLightHandler
from controller.src.handlers.end_cycle.send_to_frontend_cycle_ended_handler import SendToFrontendCycleEndedHandler

command_builder = CommandBuilder()


def test_given_no_step_when_building_then_return_empty_list():
    steps = []

    commands = command_builder.with_steps(steps).build_many()

    assert len(commands) == 0


def test_given_multiple_steps_when_building_then_list_of_length_of_steps():
    steps = [Step.WAIT_FOR_ROBOT_READY_STATE, Step.WAIT_FOR_FRONTEND_CYCLE_START]

    commands = command_builder.with_steps(steps).build_many()

    assert len(commands) == len(steps)


def test_given_wait_for_robot_ready_state_step_when_building_then_return_associated_command():
    step = Step.WAIT_FOR_ROBOT_READY_STATE
    handler_classes = [WaitForRobotReadyStateHandler]

    given_single_step_when_building_then_return_associated_command(step, handler_classes)


def test_given_wait_for_frontend_cycle_start_step_when_building_then_return_associated_command():
    step = Step.WAIT_FOR_FRONTEND_CYCLE_START
    handler_classes = [WaitForFrontendCycleStartHandler]

    given_single_step_when_building_then_return_associated_command(step, handler_classes)


def test_given_move_robot_to_resistance_station_step_when_building_then_return_associated_command():
    step = Step.MOVE_ROBOT_TO_RESISTANCE_STATION
    handler_classes = [MoveRobotToResistanceStationHandler]

    given_single_step_when_building_then_return_associated_command(step, handler_classes)


def test_given_wait_for_robot_arrival_step_when_building_then_return_associated_command():
    step = Step.WAIT_FOR_ROBOT_ARRIVAL
    handler_classes = [WaitForRobotArrivalHandler]

    given_single_step_when_building_then_return_associated_command(step, handler_classes)


def test_given_read_resistance_step_when_building_then_return_associated_command():
    step = Step.READ_RESISTANCE
    handler_classes = [ReadResistanceHandler]

    given_single_step_when_building_then_return_associated_command(step, handler_classes)


def test_given_move_robot_to_command_panel_step_when_building_then_return_associated_command():
    step = Step.MOVE_ROBOT_TO_COMMAND_PANEL
    handler_classes = [MoveRobotToCommandPanelHandler]

    given_single_step_when_building_then_return_associated_command(step, handler_classes)


def test_given_read_letters_step_when_building_then_return_associated_command():
    step = Step.READ_LETTERS
    handler_classes = [ReadLettersHandler]

    given_single_step_when_building_then_return_associated_command(step, handler_classes)


def test_given_move_robot_to_command_panel_step_when_building_then_return_associated_command():
    step = Step.MOVE_ROBOT_TO_NEXT_PUCK
    handler_classes = [MoveRobotToNextPuckHandler]

    given_single_step_when_building_then_return_associated_command(step, handler_classes)


def test_given_grip_puck_step_when_building_then_return_associated_command():
    step = Step.GRIP_PUCK
    handler_classes = [GripPuckHandler]

    given_single_step_when_building_then_return_associated_command(step, handler_classes)


def test_given_get_next_corner_position_step_when_building_then_return_associated_command():
    step = Step.GET_NEXT_CORNER_POSITION
    handler_classes = [GetNextCornerPositionHandler]

    given_single_step_when_building_then_return_associated_command(step, handler_classes)


def test_given_release_puck_step_when_building_then_return_associated_command():
    step = Step.RELEASE_PUCK
    handler_classes = [ReleasePuckHandler, SendToFrontendPuckReleasedStateHandler]

    given_single_step_when_building_then_return_associated_command(step, handler_classes)


def test_given_get_start_square_center_position_step_when_building_then_return_associated_command():
    step = Step.GET_START_SQUARE_CENTER_POSITION
    handler_classes = [GetStartSquareCenterPositionHandler]

    given_single_step_when_building_then_return_associated_command(step, handler_classes)


def test_given_end_cycle_step_when_building_then_return_associated_command():
    step = Step.END_CYCLE
    handler_classes = [TurnOnRedLightHandler, SendToFrontendCycleEndedHandler]

    given_single_step_when_building_then_return_associated_command(step, handler_classes)


def test_given_move_robot_step_when_building_then_return_associated_command():
    step = Step.MOVE_ROBOT
    handler_classes = [
        GetRobotPositionHandler,
        CalculateTrajectoryHandler,
        SendToRobotPlannedTrajectoryHandler,
        SendToFrontendPlannedTrajectoryHandler,
        SendToFrontendRealTrajectoryCoordinateHandler,
        WaitForRobotArrivalHandler
    ]

    given_single_step_when_building_then_return_associated_command(step, handler_classes)


def given_single_step_when_building_then_return_associated_command(step, handler_classes):
    commands = command_builder.some_commands().with_steps([step]).build_many()

    assert len(commands) == 1
    assert len(commands[0].handlers) == len(handler_classes)

    for idx, handler_class in enumerate(handler_classes):
        assert isinstance(commands[0].handlers[idx], handler_class)
