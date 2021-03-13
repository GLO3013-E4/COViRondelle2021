import rospy
from std_msgs.msg import Bool


# TODO : Remove this mock
if __name__ == '__main__':
    ready_publisher = rospy.Publisher('ready', Bool, queue_size=10)

    rospy.init_node('mock_ready', anonymous=True)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        ready_publisher.publish(True)
        rate.sleep()
