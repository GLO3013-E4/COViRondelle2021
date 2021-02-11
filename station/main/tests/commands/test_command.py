from unittest.mock import call

from station.main.src.commands.command import Command
from station.main.src.handlers.handler import Handler

ONCE_HANDLED_DATA = 'ONCE_HANDLED_DATA'


class StubHandler(Handler):
    def __init__(self, on_handle):
        self.on_handle = on_handle

    def handle(self, handled_data=None):
        self.on_handle(handled_data)

        if handled_data is None:
            return ONCE_HANDLED_DATA

        return None


def test_when_executing_then_handle(mocker):
    stub = mocker.stub(name='on_handle_stub')
    command = Command(StubHandler(stub))

    command.execute()

    stub.assert_called_once_with(None)


def test_given_next_command_when_executing_then_pass_handled_data(mocker):
    stub = mocker.stub(name='on_handle_stub')
    command = Command(StubHandler(stub), Command(StubHandler(stub)))

    command.execute()

    stub.assert_has_calls([call(None), call(ONCE_HANDLED_DATA)])
