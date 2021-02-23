from controller.src.commands.command import Command
from controller.src.commands.step import Step

from controller.src.handlers.wait_for_ready_state_handler import WaitForReadyStateHandler
from controller.src.handlers.send_ready_state_handler import SendReadyStateHandler
from controller.src.handlers.send_table_image.capture_table_image_handler \
    import CaptureTableImageHandler
from controller.src.handlers.send_table_image.send_table_image_handler import SendTableImageHandler


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
        if step == Step.SendReadyState:
            self._commands.append(Command([SendReadyStateHandler()]))
        if step == Step.SendTableImage:
            self._commands.append(Command([CaptureTableImageHandler(), SendTableImageHandler()]))
        # TODO : Implement rest of steps

    def build_many(self):
        return self._commands
