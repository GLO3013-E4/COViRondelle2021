class Command:
    def __init__(self, handler, next_command=None):
        self.handler = handler
        self.next_command = next_command

    # TODO : Send handled data to next command
    def execute(self):
        self.handler.handle()

        if self.next_command:
            self.next_command.execute()
