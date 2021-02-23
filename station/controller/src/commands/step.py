from enum import Enum


class Step(Enum):
    WAIT_FOR_READY_STATE = 'WaitForReadyState'
    SEND_READY_STATE = 'SendReadyState'
    SEND_TABLE_IMAGE = 'SendTableImage'
    GET_RESISTANCE_STATION_POSITION = 'GetResistanceStationPosition'
    MOVE_ROBOT = 'MoveRobot'
    READ_RESISTANCE = 'ReadResistance'
    MAP_RESISTANCE_TO_PUCK_COLORS = 'MapResistanceToPuckColors'
    GET_COMMAND_PANEL_POSITION = 'GetCommandPanelPosition'
    READ_LETTERS = 'ReadLetters'
    MAP_LETTERS_TO_PUCK_CORNERS = 'MapLettersToPuckCorners'
    GET_NEXT_PUCK_POSITION = 'GetNextPuckPosition'
    GRIP_PUCK = 'GripPuck'
    GET_NEXT_CORNER_POSITION = 'GetNextCornerPosition'
    RELEASE_PUCK = 'ReleasePuck'
    GET_START_SQUARE_CENTER_POSITION = 'GetStartSquareCenterPosition'
    END_CYCLE = 'EndCycle'
