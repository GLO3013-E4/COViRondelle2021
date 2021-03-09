import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Quaternion
import tf

sauce = tf.transformations.quaternion_from_euler(0, 3*np.pi/2, 0, 'szyx')
sauce_qui_peut = Quaternion(*sauce)

def create_robot_pose():
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.seq = 1
    pose.header.stamp = rospy.Time.now()
    
    pose.pose.position.x = 0.2
    pose.pose.position.y = 2.4
    pose.pose.orientation = sauce_qui_peut
    return pose


def create_obstacles_poses():
    pose_array = PoseArray()

    pose1 = Pose()
    pose1.position.x = 3.20
    pose1.position.y = 2.40
    pose1.orientation = sauce_qui_peut

    pose2 = Pose()
    pose2.position.x = 2.00
    pose2.position.y = 4.80
    pose2.orientation = sauce_qui_peut

    pose_array.header.frame_id = "map"
    pose_array.header.stamp = rospy.Time.now()
    pose_array.header.seq = 1
    pose_array.poses = [pose1, pose2]
    return pose_array


def create_pucks_poses():
    pose_array = PoseArray()

    pose1 = Pose()
    pose1.position.x = 6.3
    pose1.position.y = 0.2
    pose1.orientation = sauce_qui_peut

    pose2 = Pose()
    pose2.position.x = 6.3
    pose2.position.y = 4.3
    pose2.orientation = sauce_qui_peut

    pose_array.header.frame_id = "map"
    pose_array.header.stamp = rospy.Time.now()
    pose_array.header.seq = 1
    pose_array.poses = [pose1, pose2]
    return pose_array


def create_goal_pose():
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.seq = 1
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = 6.3
    pose.pose.position.y = 2.4
    pose.pose.orientation = sauce_qui_peut
    return pose


if __name__ == '__main__':

    robot_publisher = rospy.Publisher('robot', PoseStamped, queue_size=10)
    goal_publisher = rospy.Publisher('goal', PoseStamped, queue_size=10)
    obstacles_publisher = rospy.Publisher('obstacles', PoseArray, queue_size=10)
    pucks_publisher = rospy.Publisher('pucks', PoseArray, queue_size=10)

    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        robot_publisher.publish(create_robot_pose())
        obstacles_publisher.publish(create_obstacles_poses())
        pucks_publisher.publish(create_pucks_poses())
        goal_publisher.publish(create_goal_pose())
        rate.sleep()
