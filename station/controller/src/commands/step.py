from enum import Enum


class Step(Enum):
    WaitForReadyState = 'WaitForReadyState'
    SendReadyState = 'SendReadyState'
    SendTableImage = 'SendTableImage'
    GetResistanceStationPosition = 'GetResistanceStationPosition'
    MoveRobot = 'MoveRobot'
    ReadResistance = 'ReadResistance'
    MapResistanceToPuckColors = 'MapResistanceToPuckColors'
    GetCommandPanelPosition = 'GetCommandPanelPosition'
    ReadLetters = 'ReadLetters'
    MapLettersToPuckCorners = 'MapLettersToPuckCorners'
    GetNextPuckPosition = 'GetNextPuckPosition'
    GripPuck = 'GripPuck'
    GetNextCornerPosition = 'GetNextCornerPosition'
    ReleasePuck = 'ReleasePuck'
    GetStartSquareCenterPosition = 'GetStartSquareCenterPosition'
    EndCycle = 'EndCycle'
