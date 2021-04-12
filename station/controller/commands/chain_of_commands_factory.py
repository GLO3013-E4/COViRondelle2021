from commands.cycle_steps import cycle_steps


class ChainOfCommandsFactory:
    def __init__(self, command_builder):
        self.command_builder = command_builder

    def create(self, steps = cycle_steps):
        return self.command_builder.some_commands().with_steps(steps).build_many()
