from main.src.commands.command_factory import CommandFactory
from main.src.commands.read_image_handler import ReadImageHandler

command_factory = CommandFactory()


def test_when_creating_then_add_read_image_command():
    commands = command_factory.create()

    assert isinstance(commands[0].handler, ReadImageHandler)
    assert commands[0].next_command is None
