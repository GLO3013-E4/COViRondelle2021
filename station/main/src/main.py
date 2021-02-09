#!/usr/bin/env python

# TODO : Install / define rospy somehow
# pylint: disable=import-error
import rospy
from std_msgs.msg import String


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


def sample_rospy():
    rospy.init_node('station', anonymous=True)
    rospy.Subscriber('chatter', String, callback)

    rospy.spin()


if __name__ == '__main__':
    sample_rospy()
