class Command:
    def __init__(self, handler, next_command=None):
        self.handler = handler
        self.next_command = next_command

    def execute(self, handled_data=None):
        handled_data = self.handler.handle(handled_data)

        if self.next_command:
            self.next_command.execute(handled_data)
