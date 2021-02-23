from controller.src.commands.command import Command
from controller.src.commands.step import Step


# TODO : Test CommandBuilder
class CommandBuilder:
    _commands = []

    def with_steps(self, steps):
        self._commands = []

        for step in steps:
            self._with_step(step)

        return self

    def _with_step(self, step):
        if step == Step.WaitForReadyState:
            self._commands.append(Command([]))  # TODO : Add WaitForReadyStateHandler
        if step == Step.SendReadyState:
            self._commands.append(Command([]))  # TODO : Add SendReadyStateHandler
        # TODO : Implement rest of steps

    def build_many(self):
        return self._commands
