from main.src.commands.command import Command
from main.src.handlers.handler import Handler


class StubHandler(Handler):
    def __init__(self, on_handle):
        self.on_handle = on_handle

    def handle(self):
        self.on_handle()


class StubCommand(Command):
    def __init__(self, on_execute):
        self.on_execute = on_execute

    def execute(self):
        self.on_execute()


def test_when_executing_then_handle(mocker):
    stub = mocker.stub(name='on_handle_stub')
    handler = StubHandler(stub)
    command = Command(handler)

    command.execute()

    stub.assert_called_once()


def test_given_next_command_when_executing_then_execute_next_command(mocker):
    stub = mocker.stub(name='on_execute_stub')
    next_command = StubCommand(stub)
    command = Command(Handler(), next_command)

    command.execute()

    stub.assert_called_once()
