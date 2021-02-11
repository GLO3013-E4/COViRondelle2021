#!/usr/bin/env python
import rospy
from std_msgs.msg import String

from robot.main.src.commands.command_factory import CommandFactory
from robot.main.src.handlers.read_image_handler import ReadImageHandler
from robot.main.src.handlers.map_letters_handler import MapLettersHandler
from robot.main.src.mappers.letter_mapper import LetterMapper
from robot.main.src.readers.image_reader import ImageReader


def robot():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('robot', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()


# TODO : This is an example. It's how we will chain commands.
def start_chain_of_commands():
    first_command = CommandFactory().create([
        ReadImageHandler(ImageReader(), 'src/data/images/command_panel_example_1.png'),
        MapLettersHandler(LetterMapper())
    ])

    first_command.execute()


if __name__ == '__main__':
    try:
        robot()
    except rospy.ROSInterruptException:
        pass

    start_chain_of_commands()
