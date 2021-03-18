import random
import rospy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Path


def create_path():
    poses_amount = random.randint(5, 20)
    path = Path()

    for pose in range(0, poses_amount):
        pose = Pose()
        pose.position.x = random.randint(100, 1500)
        pose.position.y = random.randint(100, 814)

        path.poses.append(pose)

    return path


# TODO : Remove this mock
if __name__ == '__main__':
    robot_publisher = rospy.Publisher('path', Pose, queue_size=10)

    rospy.init_node('mock_path', anonymous=True)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        robot_publisher.publish(create_path())
        rate.sleep()
