from controller.src.commands.step import Step

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
