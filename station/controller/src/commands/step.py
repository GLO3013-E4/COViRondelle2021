from enum import Enum


class Step(Enum):
    WAIT_FOR_ROBOT_READY_STATE = 'WaitForRobotReadyState'
    WAIT_FOR_FRONTEND_CYCLE_START = 'WaitForFrontendCycleStart'
    MOVE_ROBOT_TO_RESISTANCE_STATION = 'MoveRobotToResistanceStation'
    WAIT_FOR_ROBOT_ARRIVAL = 'WaitForRobotArrival'
    READ_RESISTANCE = 'ReadResistance'
    MOVE_ROBOT_TO_COMMAND_PANEL = 'MoveRobotToCommandPanel'
    READ_LETTERS = 'ReadLetters'
    MOVE_ROBOT_TO_NEXT_PUCK = 'MoveRobotToNextPuck'
    # TODO : Rework steps
    GRIP_PUCK = 'GripPuck'
    GET_NEXT_CORNER_POSITION = 'GetNextCornerPosition'
    RELEASE_PUCK = 'ReleasePuck'
    GET_START_SQUARE_CENTER_POSITION = 'GetStartSquareCenterPosition'
    END_CYCLE = 'EndCycle'
    # TODO : Remove MOVE_ROBOT
    MOVE_ROBOT = 'MoveRobot'
