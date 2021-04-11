class Command:
    def __init__(self, handlers):
        self.handlers = handlers

    def execute(self, handled_data):
        for handler in self.handlers:

            handled_data = handler.handle(handled_data)

            handler.unregister()

        return handled_data
