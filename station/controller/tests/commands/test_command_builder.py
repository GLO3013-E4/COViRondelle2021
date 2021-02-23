from controller.src.commands.command_builder import CommandBuilder

command_builder = CommandBuilder()


def test_given_no_step_then_return_empty_list():
    steps = []

    commands = command_builder.with_steps(steps).build_many()

    assert len(commands) is 0
