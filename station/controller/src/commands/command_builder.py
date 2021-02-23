from controller.src.commands.command import Command
from controller.src.commands.step import Step

from controller.src.handlers.wait_for_ready_state_handler import WaitForReadyStateHandler
from controller.src.handlers.send_ready_state_handler import SendReadyStateHandler
from controller.src.handlers.send_table_image.capture_table_image_handler import CaptureTableImageHandler
from controller.src.handlers.send_table_image.send_table_image_handler import SendTableImageHandler
from controller.src.handlers.get_resistance_station_position_handler import GetResistanceStationPositionHandler
from controller.src.handlers.move_robot.get_robot_position_handler import GetRobotPositionHandler
from controller.src.handlers.move_robot.calculate_trajectory_handler import CalculateTrajectoryHandler
from controller.src.handlers.move_robot.send_planned_trajectory_handler import SendPlannedTrajectoryHandler
from controller.src.handlers.move_robot.send_real_trajectory_coordinate_handler import SendRealTrajectoryCoordinateHandler
from controller.src.handlers.move_robot.wait_for_robot_arrival_handler import WaitForRobotArrivalHandler
from controller.src.handlers.read_resistance_handler import ReadResistanceHandler


class CommandBuilder:
    _commands = []

    def with_steps(self, steps):
        self._commands = []

        for step in steps:
            self._with_step(step)

        return self

    def _with_step(self, step):
        if step == Step.WaitForReadyState:
            self._commands.append(Command([WaitForReadyStateHandler()]))
        elif step == Step.SendReadyState:
            self._commands.append(Command([SendReadyStateHandler()]))
        elif step == Step.SendTableImage:
            self._commands.append(Command([CaptureTableImageHandler(), SendTableImageHandler()]))
        elif step == Step.GetResistanceStationPosition:
            self._commands.append(Command([GetResistanceStationPositionHandler()]))
        elif step == Step.MoveRobot:
            self._commands.append(Command([
                GetRobotPositionHandler(),
                CalculateTrajectoryHandler(),
                SendPlannedTrajectoryHandler(),
                SendRealTrajectoryCoordinateHandler(),
                WaitForRobotArrivalHandler()
            ]))
        elif step == Step.ReadResistance:
            self._commands.append(Command([ReadResistanceHandler()]))
        # TODO : Implement rest of steps

    def build_many(self):
        return self._commands
