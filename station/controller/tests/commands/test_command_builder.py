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
    steps = [Step.WaitForReadyState]

    commands = command_builder.with_steps(steps).build_many()

    assert len(commands) == 1
    assert len(commands[0].handlers) == 1
    assert isinstance(commands[0].handlers[0], WaitForReadyStateHandler)


def test_given_send_ready_state_step_when_building_then_return_send_ready_command():
    steps = [Step.SendReadyState]

    commands = command_builder.with_steps(steps).build_many()

    assert len(commands) == 1
    assert len(commands[0].handlers) == 1
    assert isinstance(commands[0].handlers[0], SendReadyStateHandler)


def test_given_send_table_image_step_when_building_then_return_send_table_image_command():
    steps = [Step.SendTableImage]

    commands = command_builder.with_steps(steps).build_many()

    assert len(commands) == 1
    assert len(commands[0].handlers) == 2
    assert isinstance(commands[0].handlers[0], CaptureTableImageHandler)
    assert isinstance(commands[0].handlers[1], SendTableImageHandler)


def test_given_get_resistance_station_position_step_when_building_then_return_get_resistance_station_position_command():
    steps = [Step.GetResistanceStationPosition]

    commands = command_builder.with_steps(steps).build_many()

    assert len(commands) == 1
    assert len(commands[0].handlers) == 1
    assert isinstance(commands[0].handlers[0], GetResistanceStationPositionHandler)


def test_given_move_robot_step_when_building_then_return_move_robot_command():
    steps = [Step.MoveRobot]

    commands = command_builder.with_steps(steps).build_many()

    assert len(commands) == 1
    assert len(commands[0].handlers) == 5
    assert isinstance(commands[0].handlers[0], GetRobotPositionHandler)
    assert isinstance(commands[0].handlers[1], CalculateTrajectoryHandler)
    assert isinstance(commands[0].handlers[2], SendPlannedTrajectoryHandler)
    assert isinstance(commands[0].handlers[3], SendRealTrajectoryCoordinateHandler)
    assert isinstance(commands[0].handlers[4], WaitForRobotArrivalHandler)
