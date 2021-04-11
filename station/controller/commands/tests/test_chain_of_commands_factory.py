from commands.step import Step
from commands.cycle_steps import cycle_steps
from commands.command import Command
from commands.command_builder import CommandBuilder
from commands.chain_of_commands_factory import ChainOfCommandsFactory

sent_steps = [Step.GRIP_PUCK]
command_for_sent_steps = Command([])
first_command = Command([])
second_command = Command([])
third_command = Command([])


class MockCommandBuilder(CommandBuilder):
    has_sent_steps = False

    def some_commands(self):
        return self

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


def test_when_creating_then_return_all_commands():
    commands = chain_of_commands_factory.create()

    assert commands == [first_command, second_command, third_command]


def test_given_steps_when_creating_then_return_command_for_sent_steps():
    commands = chain_of_commands_factory.create(sent_steps)

    assert commands == [command_for_sent_steps]
