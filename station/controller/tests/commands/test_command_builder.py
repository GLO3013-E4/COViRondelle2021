from controller.src.commands.step import Step
from controller.src.commands.command_builder import CommandBuilder

from controller.src.handlers.wait_for_ready_state_handler import WaitForReadyStateHandler
from controller.src.handlers.send_ready_state_handler import SendReadyStateHandler

command_builder = CommandBuilder()


def test_given_no_step_when_building_then_return_empty_list():
    steps = []

    commands = command_builder.with_steps(steps).build_many()

    assert len(commands) is 0


def test_given_multiple_steps_when_building_then_list_of_length_of_steps():
    steps = [Step.WaitForReadyState, Step.SendReadyState]

    commands = command_builder.with_steps(steps).build_many()

    assert len(commands) is len(steps)


def test_given_wait_for_ready_state_step_when_building_then_return_wait_for_ready_step_command():
    steps = [Step.WaitForReadyState]

    commands = command_builder.with_steps(steps).build_many()

    assert len(commands) is 1
    assert len(commands[0].handlers) is 1
    assert isinstance(commands[0].handlers[0], WaitForReadyStateHandler)


def test_given_send_ready_state_step_when_building_then_return_send_ready_step_command():
    steps = [Step.SendReadyState]

    commands = command_builder.with_steps(steps).build_many()

    assert len(commands) is 1
    assert len(commands[0].handlers) is 1
    assert isinstance(commands[0].handlers[0], SendReadyStateHandler)
