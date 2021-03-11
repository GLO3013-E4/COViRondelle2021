import rospy
from geometry_msgs.msg import Pose


def create_robot_pose():
    pose = Pose()
    pose.position.x = 20
    pose.position.y = 240
    return pose


# TODO : Remove this mock
if __name__ == '__main__':
    robot_publisher = rospy.Publisher('robot', Pose, queue_size=10)

    rospy.init_node('mock_robot', anonymous=True)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        robot_publisher.publish(create_robot_pose())
        rate.sleep()
