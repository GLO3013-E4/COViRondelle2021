#!/usr/bin/env python

# TODO : Install / define rospy somehow
# pylint: disable=import-error
import rospy
from std_msgs.msg import String

from main.src.commands.command_factory import CommandFactory
from main.src.handlers.read_image_handler import ReadImageHandler

from main.src.readers.image_reader import ImageReader


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


def sample_rospy():
    rospy.init_node('station', anonymous=True)
    rospy.Subscriber('chatter', String, callback)

    rospy.spin()


# TODO : This is an example. It's how we will chain commands.
def start_chain_of_commands():
    first_command = CommandFactory().create([
        ReadImageHandler(ImageReader(), 'src/data/images/command_panel_example_1.png')
    ])

    first_command.execute()


if __name__ == '__main__':
    sample_rospy()

    start_chain_of_commands()
