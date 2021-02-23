import pytest

from controller.src.commands.chain_of_commands_factory import ChainOfCommandsFactory
from controller.src.handlers.handler import Handler

chain_of_commands_factory = ChainOfCommandsFactory()


def test_given_no_handler_when_creating_then_raise_exception():
    with pytest.raises(Exception):
        chain_of_commands_factory.create([])


def test_given_one_handler_when_creating_then_add_handler_to_command():
    handler = Handler()

    command = chain_of_commands_factory.create([handler])

    assert command.handler is handler
    assert command.next_command is None


def test_given_multiple_handlers_when_creating_then_add_handlers_to_commands():
    first_handler = Handler()
    second_handler = Handler()

    command = chain_of_commands_factory.create([first_handler, second_handler])

    assert command.handler is first_handler
    assert command.next_command is not None
    assert command.next_command.handler is second_handler
    assert command.next_command.next_command is None
