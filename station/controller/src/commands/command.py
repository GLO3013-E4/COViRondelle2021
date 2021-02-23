class Command:
    def __init__(self, handlers, next_command=None):
        self.handlers = handlers
        self.next_command = next_command

    def execute(self, handled_data=None):
        for handler in self.handlers:
            handled_data = handler.handle(handled_data)

        if self.next_command:
            self.next_command.execute(handled_data)
