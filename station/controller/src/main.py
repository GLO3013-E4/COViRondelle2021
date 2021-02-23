#!/usr/bin/env python

from main.src.commands.command_factory import CommandFactory


if __name__ == '__main__':
    first_command = CommandFactory().create()

    first_command.execute()
