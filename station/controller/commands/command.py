class Command:
    next_command = None

    def __init__(self, handlers):
        self.handlers = handlers

    def execute(self, handled_data=None):
        for handler in self.handlers:
            is_finished = False

            while not is_finished:
                handled_data, is_finished = handler.handle(handled_data)

            handler.unregister()

        if self.next_command:
            self.next_command.execute(handled_data)
