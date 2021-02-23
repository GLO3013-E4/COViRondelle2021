#!/usr/bin/env python

from controller.src.commands.command_builder import CommandBuilder
from controller.src.commands.chain_of_commands_factory import ChainOfCommandsFactory


def create_chain_of_commands():
    command_builder = CommandBuilder()
    chain_of_commands_factory = ChainOfCommandsFactory(command_builder)

    return chain_of_commands_factory.create()


if __name__ == '__main__':
    first_command = create_chain_of_commands()

    first_command.execute()
