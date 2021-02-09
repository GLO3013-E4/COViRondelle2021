from main.src.commands.command import Command


class CommandFactory:
    # TODO : Receive list of handlers
    @staticmethod
    def create(handlers):
        if len(handlers) == 0:
            raise Exception('No handler provided')

        command = Command(handlers.pop())

        for handler in handlers[::-1]:
            command = Command(handler, command)

        return command
