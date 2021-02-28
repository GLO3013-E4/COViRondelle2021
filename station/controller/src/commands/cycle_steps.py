from controller.src.commands.step import Step

cycle_steps = [
    Step.WAIT_FOR_ROBOT_READY_STATE,
    Step.WAIT_FOR_FRONTEND_CYCLE_START,
    Step.MOVE_ROBOT_TO_RESISTANCE_STATION,
    Step.WAIT_FOR_ROBOT_ARRIVAL,
    Step.READ_RESISTANCE,
    Step.MOVE_ROBOT_TO_COMMAND_PANEL,
    Step.WAIT_FOR_ROBOT_ARRIVAL,
    Step.READ_LETTERS,
    Step.MOVE_ROBOT_TO_NEXT_PUCK,
    Step.WAIT_FOR_ROBOT_ARRIVAL,
    Step.GRIP_PUCK,
    # TODO : Rework cycle steps
    Step.GET_NEXT_CORNER_POSITION,
    Step.MOVE_ROBOT,
    Step.RELEASE_PUCK,
    Step.MOVE_ROBOT_TO_NEXT_PUCK,
    Step.WAIT_FOR_ROBOT_ARRIVAL,
    Step.GRIP_PUCK,
    Step.GET_NEXT_CORNER_POSITION,
    Step.MOVE_ROBOT,
    Step.RELEASE_PUCK,
    Step.MOVE_ROBOT_TO_NEXT_PUCK,
    Step.WAIT_FOR_ROBOT_ARRIVAL,
    Step.GRIP_PUCK,
    Step.GET_NEXT_CORNER_POSITION,
    Step.MOVE_ROBOT,
    Step.RELEASE_PUCK,
    Step.GET_START_SQUARE_CENTER_POSITION,
    Step.END_CYCLE
]
