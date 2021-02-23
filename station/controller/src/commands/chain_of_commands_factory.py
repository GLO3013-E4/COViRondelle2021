from controller.src.commands.step import Step


class ChainOfCommandsFactory:
    # TODO : Move cycle_steps definition
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
        Step.ReadLetters,
        Step.MapLettersToPuckCorners,
        Step.GetNextPuckPosition,
        Step.MoveRobot,
        Step.GripPuck,
        Step.GetNextCornerPosition,
        Step.MoveRobot,
        Step.ReleasePuck,
        Step.GetNextPuckPosition,
        Step.MoveRobot,
        Step.GripPuck,
        Step.GetNextCornerPosition,
        Step.MoveRobot,
        Step.ReleasePuck,
        Step.GetNextPuckPosition,
        Step.MoveRobot,
        Step.GripPuck,
        Step.GetNextCornerPosition,
        Step.MoveRobot,
        Step.ReleasePuck,
        Step.GetStartSquareCenterPosition,
        Step.EndCycle
    ]

    def __init__(self, command_builder):
        self.command_builder = command_builder

    # TODO : Change to create(self, steps=cycle_steps) for testing purposes
    def create(self):
        commands = self.command_builder.with_steps(self.cycle_steps).build_many()

        next_command = commands.pop()

        for command in commands[::-1]:
            command.next_command = next_command
            next_command = command

        return next_command
