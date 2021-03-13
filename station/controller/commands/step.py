from enum import Enum


class Step(Enum):
    WAIT_FOR_ROBOT_READY_STATE = 'WaitForRobotReadyState'
    WAIT_FOR_FRONTEND_CYCLE_START = 'WaitForFrontendCycleStart'
    MOVE_ROBOT_TO_RESISTANCE_STATION = 'MoveRobotToResistanceStation'
    READ_RESISTANCE = 'ReadResistance'
    MOVE_ROBOT_TO_COMMAND_PANEL = 'MoveRobotToCommandPanel'
    READ_LETTERS = 'ReadLetters'
    MOVE_ROBOT_TO_NEXT_PUCK = 'MoveRobotToNextPuck'
    GRIP_PUCK = 'GripPuck'
    MOVE_ROBOT_TO_NEXT_CORNER = 'MoveRobotToNextCorner'
    RELEASE_PUCK = 'ReleasePuck'
    MOVE_ROBOT_TO_SQUARE_CENTER = 'MoveRobotToSquareCenter'
    END_CYCLE = 'EndCycle'
