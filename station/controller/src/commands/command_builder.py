from controller.src.commands.command import Command
from controller.src.commands.step import Step


class CommandBuilder:
    commands = []

    def with_steps(self, steps):
        for step in steps:
            self._with_step(step)

    def _with_step(self, step):
        if step == Step.WaitForReadyState:
            self.commands.append(Command([]))  # TODO : Add WaitForReadyStateHandler
        if step == Step.SendReadyState:
            self.commands.append(Command([]))  # TODO : Add SendReadyStateHandler

    def build_many(self):
        return self.commands
