import rospy
from geometry_msgs.msg import PoseStamped, Pose, PoseArray


def create_robot_pose():
    pose = PoseStamped()
    pose.pose.position.x = 20
    pose.pose.position.y = 240
    pose.header = "map"
    return pose


def create_obstacles_poses():
    pose_array = PoseArray()

    pose1 = Pose()
    pose1.position.x = 320
    pose1.position.y = 240

    pose2 = Pose()
    pose2.position.x = 200
    pose2.position.y = 480

    pose_array.poses = [pose1, pose2]
    return pose_array


def create_pucks_poses():
    pose_array = PoseArray()

    pose1 = Pose()
    pose1.position.x = 630
    pose1.position.y = 20

    pose2 = Pose()
    pose2.position.x = 630
    pose2.position.y = 460

    pose_array.poses = [pose1, pose2]
    return pose_array


def create_goal_pose():
    pose = PoseStamped()
    pose.pose.position.x = 1200
    pose.pose.position.y = 450
    return pose


# TODO : Remove this mock (and usage)
if __name__ == '__main__':

    goal_publisher = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)

    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        goal_publisher.publish(create_goal_pose())
        rate.sleep()
