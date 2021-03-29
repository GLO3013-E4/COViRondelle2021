from geometry_msgs.msg import Pose, PoseArray


def create_robot_pose():
    pose = Pose()
    pose.position.x = 20
    pose.position.y = 240
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
    pose = Pose()
    pose.position.x = 630
    pose.position.y = 240
    return pose


# TODO : Remove this mock (and usage)
if __name__ == '__main__':
    pass
