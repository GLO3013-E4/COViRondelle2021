from unittest.mock import call

from controller.src.commands.command import Command
from controller.src.handlers.handler import Handler

ONCE_HANDLED_DATA = 'ONCE_HANDLED_DATA'


class StubHandler(Handler):
    def __init__(self, on_handle, on_unregister):
        self.on_handle = on_handle
        self.on_unregister = on_unregister

    def handle(self, handled_data=None):
        self.on_handle(handled_data)

        if handled_data is None:
            return ONCE_HANDLED_DATA, True

        return None, True

    def unregister(self):
        self.on_unregister()


def test_when_executing_then_handle(mocker):
    on_handle_stub = mocker.stub(name='on_handle_stub')
    on_unregister_stub = mocker.stub(name='on_unregister_stub')
    command = Command([StubHandler(on_handle_stub, on_unregister_stub)])

    command.execute()

    on_handle_stub.assert_called_once_with(None)
    on_unregister_stub.assert_called_once()


def test_given_multiple_handlers_when_executing_then_handle(mocker):
    on_handle_stub = mocker.stub(name='on_handle_stub')
    on_unregister_stub = mocker.stub(name='on_unregister_stub')
    command = Command([
        StubHandler(on_handle_stub, on_unregister_stub),
        StubHandler(on_handle_stub, on_unregister_stub)
    ])

    command.execute()

    on_handle_stub.assert_has_calls([call(None), call(ONCE_HANDLED_DATA)])
    assert on_unregister_stub.call_count == 2


def test_given_next_command_when_executing_then_pass_handled_data(mocker):
    on_handle_stub = mocker.stub(name='on_handle_stub')
    on_unregister_stub = mocker.stub(name='on_unregister_stub')
    command = Command([StubHandler(on_handle_stub, on_unregister_stub)])
    command.next_command = Command([StubHandler(on_handle_stub, on_unregister_stub)])

    command.execute()

    on_handle_stub.assert_has_calls([call(None), call(ONCE_HANDLED_DATA)])
    assert on_unregister_stub.call_count == 2
