from controller.src.commands.step import Step


class ChainOfCommandsFactory:
    steps = [
        Step.WaitForReadyState,
        Step.SendReadyState,
        # TODO : Add rest of steps
    ]

    def __init__(self, command_builder):
        self.command_builder = command_builder

    def create(self):
        commands = self.command_builder.with_steps(self.steps).build_many()

        next_command = commands.pop()

        for command in commands[::-1]:
            command.next_command = next_command
            next_command = command

        return next_command
