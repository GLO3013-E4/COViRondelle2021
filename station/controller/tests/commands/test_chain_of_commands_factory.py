from controller.src.commands.command import Command
from controller.src.commands.command_builder import CommandBuilder
from controller.src.commands.chain_of_commands_factory import ChainOfCommandsFactory

first_command = Command([])
second_command = Command([])
third_command = Command([])


class MockCommandBuilder(CommandBuilder):
    def with_steps(self, steps):
        if steps is ChainOfCommandsFactory.steps:
            return self

        return None

    @staticmethod
    def build_many():
        return [first_command, second_command, third_command]


chain_of_commands_factory = ChainOfCommandsFactory(MockCommandBuilder())


def test_when_creating_then_return_first_command():
    command = chain_of_commands_factory.create()

    assert command is first_command


def test_when_creating_then_return_correct_next_commands():
    command = chain_of_commands_factory.create()

    assert command.next_command is second_command
    assert command.next_command.next_command is third_command
    assert command.next_command.next_command.next_command is None
