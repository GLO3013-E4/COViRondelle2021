from enum import Enum


class Step(Enum):
    WAIT_FOR_ROBOT_READY_STATE = 'WaitForRobotReadyState'
    WAIT_FOR_FRONTEND_CYCLE_START = 'WaitForFrontendCycleStart'
    MOVE_ROBOT_TO_RESISTANCE_STATION = 'MoveRobotToResistanceStation'
    WAIT_FOR_ROBOT_ARRIVAL = 'WaitForRobotArrival'
    # TODO : Rework steps
    READ_RESISTANCE = 'ReadResistance'
    MAP_RESISTANCE_TO_PUCK_COLORS = 'MapResistanceToPuckColors'
    GET_COMMAND_PANEL_POSITION = 'GetCommandPanelPosition'
    MOVE_ROBOT = 'MoveRobot'
    READ_LETTERS = 'ReadLetters'
    MAP_LETTERS_TO_PUCK_CORNERS = 'MapLettersToPuckCorners'
    GET_NEXT_PUCK_POSITION = 'GetNextPuckPosition'
    GRIP_PUCK = 'GripPuck'
    GET_NEXT_CORNER_POSITION = 'GetNextCornerPosition'
    RELEASE_PUCK = 'ReleasePuck'
    GET_START_SQUARE_CENTER_POSITION = 'GetStartSquareCenterPosition'
    END_CYCLE = 'EndCycle'
