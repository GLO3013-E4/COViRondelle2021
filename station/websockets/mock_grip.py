import rospy
from std_msgs.msg import Bool


# TODO : Remove this mock
if __name__ == '__main__':
    grip_publisher = rospy.Publisher('grip', Bool, queue_size=10)

    rospy.init_node('mock_grip', anonymous=True)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        grip_publisher.publish(True)
        rate.sleep()
