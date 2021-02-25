from controller.src.commands.cycle_steps import cycle_steps
from controller.src.commands.step import Step


class ChainOfCommandsFactory:
    def __init__(self, command_builder):
        self.command_builder = command_builder

    # TODO : Steps should always be cycle_steps, this is for testing until the final implementation
    def create(self, steps: [Step] = cycle_steps):
        commands = self.command_builder.with_steps(steps).build_many()

        next_command = commands.pop()

        for command in commands[::-1]:
            command.next_command = next_command
            next_command = command

        return next_command
