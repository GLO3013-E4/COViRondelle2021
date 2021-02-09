from main.src.commands.command import Command
from main.src.commands.read_image_handler import ReadImageHandler


class CommandFactory:
    @staticmethod
    def create():
        read_image_command = Command(ReadImageHandler())

        return [read_image_command]
