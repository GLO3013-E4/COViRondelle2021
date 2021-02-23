from controller.src.commands.step import Step


class ChainOfCommandsFactory:
    cycle_steps = [
        Step.WaitForReadyState,
        Step.SendReadyState,
        Step.SendTableImage,
        Step.GetResistanceStationPosition,
        Step.MoveRobot,
        Step.ReadResistance,
        Step.MapResistanceToPuckColors,
        Step.GetCommandPanelPosition,
        Step.MoveRobot,
        Step.ReadLetters
        # TODO : Add rest of steps
    ]

    def __init__(self, command_builder):
        self.command_builder = command_builder

    # TODO : Add create_with_steps(self, steps) for testing purposes

    def create(self):
        commands = self.command_builder.with_steps(self.cycle_steps).build_many()

        next_command = commands.pop()

        for command in commands[::-1]:
            command.next_command = next_command
            next_command = command

        return next_command
