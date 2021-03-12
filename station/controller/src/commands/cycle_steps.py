from commands.step import Step

cycle_steps = [
    Step.WAIT_FOR_ROBOT_READY_STATE,
    Step.WAIT_FOR_FRONTEND_CYCLE_START,
    Step.MOVE_ROBOT_TO_RESISTANCE_STATION,
    Step.READ_RESISTANCE,
    Step.MOVE_ROBOT_TO_COMMAND_PANEL,
    Step.READ_LETTERS,
    Step.MOVE_ROBOT_TO_NEXT_PUCK,
    Step.GRIP_PUCK,
    Step.MOVE_ROBOT_TO_NEXT_CORNER,
    Step.RELEASE_PUCK,
    Step.MOVE_ROBOT_TO_NEXT_PUCK,
    Step.GRIP_PUCK,
    Step.MOVE_ROBOT_TO_NEXT_CORNER,
    Step.RELEASE_PUCK,
    Step.MOVE_ROBOT_TO_NEXT_PUCK,
    Step.GRIP_PUCK,
    Step.MOVE_ROBOT_TO_NEXT_CORNER,
    Step.RELEASE_PUCK,
    Step.MOVE_ROBOT_TO_SQUARE_CENTER,
    Step.END_CYCLE
]
