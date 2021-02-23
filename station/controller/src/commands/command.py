class Command:
    next_command = None

    def __init__(self, handlers):
        self.handlers = handlers

    # TODO : Why handled_data=None ?
    def execute(self, handled_data=None):
        for handler in self.handlers:
            handled_data = handler.handle(handled_data)

        if self.next_command:
            self.next_command.execute(handled_data)
