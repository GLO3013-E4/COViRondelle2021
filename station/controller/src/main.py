#!/usr/bin/env python

from main.src.commands.command_factory import CommandFactory


# TODO : Rework start_chain_of_command
def start_chain_of_commands():
    first_command = CommandFactory().create([])

    first_command.execute()


if __name__ == '__main__':
    start_chain_of_commands()
