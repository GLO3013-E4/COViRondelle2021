from controller.src.commands.step import Step
from controller.src.commands.cycle_steps import cycle_steps
from controller.src.commands.command import Command
from controller.src.commands.command_builder import CommandBuilder
from controller.src.commands.chain_of_commands_factory import ChainOfCommandsFactory

sent_steps = [Step.GRIP_PUCK]
command_for_sent_steps = Command([])
first_command = Command([])
second_command = Command([])
third_command = Command([])


class MockCommandBuilder(CommandBuilder):
    has_sent_steps = False

    def with_steps(self, steps):
        if steps is sent_steps:
            self.has_sent_steps = True
            return self

        if steps is cycle_steps:
            return self

        return None

    def build_many(self):
        if self.has_sent_steps:
            return [command_for_sent_steps]

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


def test_given_steps_when_creating_then_return_command_for_sent_steps():
    command = chain_of_commands_factory.create(sent_steps)

    assert command is command_for_sent_steps
