#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String, Bool


class RedLight:
    def __init__(self):
        rospy.init_node('red_light', anonymous=True)
        self.pub = rospy.Publisher('letters', String, queue_size=10)
        self.sub = rospy.Subscriber('read_letters', Bool, callback)


def callback(data):
    if data.data:
        #Todo
        # self.pub.publish(var)
        raise Exception


if __name__ == '__main__':
    try:
        RedLight()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
