#!/usr/bin/env python
import rospy
import json


from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from path_following.config import NODE_SIZE
from path_following.vectorizer import Vectorizer


class PathFollower:
    def __init__(self):
        self.robot = None
        self.robot_angle = None
        self.path = None
        self.node_size = NODE_SIZE
        self.vectorizer = Vectorizer()

        self.pub = rospy.Publisher('movement_vectors_string', String, queue_size=10)
        rospy.Subscriber('robot', Pose, self.callback_robot)
        rospy.Subscriber('path', PoseArray, self.callback_path)

    def callback_robot(self, data):
        robot_dict = json.loads(data.data)

        self.vectorizer.set_robot_position(robot_dict["robot"])
        self.vectorizer.set_robot_angle(robot_dict["angle"])

    def callback_path(self, data):
        nodes = json.loads(data.data)
        vectors = self.vectorizer.path_to_vectors(nodes)
        self.pub.publish(json.dumps(vectors))


def path_follower():
    rospy.init_node('path_following', anonymous=True)

    pathfollower = PathFollower()

    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    try:
        path_follower()
    except rospy.ROSInterruptException:
        pass
