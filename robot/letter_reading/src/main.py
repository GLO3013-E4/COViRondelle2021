#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String, Bool


class LetterReading:
    def __init__(self):
        rospy.init_node('letter_reading', anonymous=True)
        self.pub = rospy.Publisher('letters', String, queue_size=10)
        self.sub = rospy.Subscriber('read_letters',Bool ,self.callback)
    def callback(self, data):
        if(data.data):
            #Todo
            # self.pub.publish(var)
            raise Exception
if __name__ == '__main__':
    try:
        LetterReading()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass