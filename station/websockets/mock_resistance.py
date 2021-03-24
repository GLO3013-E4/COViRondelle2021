import random
import rospy
from std_msgs.msg import Float32


def create_resistance():
    return random.uniform(100, 1000000)


# TODO : Remove this mock
if __name__ == '__main__':
    resistance_publisher = rospy.Publisher('resistance', Float32, queue_size=10)

    rospy.init_node('mock_resistance', anonymous=True)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        resistance_publisher.publish(create_resistance())
        rate.sleep()
