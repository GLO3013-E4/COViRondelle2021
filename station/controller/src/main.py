#!/usr/bin/env python

import rospy
from controller.src.commands.command_builder import CommandBuilder
from controller.src.commands.chain_of_commands_factory import ChainOfCommandsFactory


def create_chain_of_commands():
    command_builder = CommandBuilder()
    chain_of_commands_factory = ChainOfCommandsFactory(command_builder)

    return chain_of_commands_factory.create()


def controller():
    rospy.init_node('controller', anonymous=True)

    rate = rospy.Rate(10)

    is_started = False

    while not rospy.is_shutdown():
        if not is_started:
            first_command = create_chain_of_commands()
            first_command.execute()
            is_started = True

        rate.sleep()


if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass
