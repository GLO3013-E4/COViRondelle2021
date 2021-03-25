#!/usr/bin/env python
import rospy
import json


from std_msgs.msg import String
from nav_msgs.msg import Path
from path_following.config import NODE_SIZE
from path_following.vectorizer import Vectorizer


class PathFollower:
    def __init__(self):
        self.robot = None
        self.robot_angle = None
        self.path = []
        self.node_size = NODE_SIZE
        self.vectorizer = Vectorizer()

        self.pub = rospy.Publisher('movement_vectors_string', String, queue_size=10)
        rospy.Subscriber('robot', String, self.callback_robot)
        rospy.Subscriber('path', Path, self.callback_path)

    def callback_robot(self, data):
        robot_dict = json.loads(data.data)

        self.vectorizer.set_robot_position(robot_dict["robot"])
        self.vectorizer.set_robot_angle(robot_dict["angle"])

    def callback_path(self, data):
        nodes = []
        for coordinate in data.poses:
            x = coordinate.pose.position.x
            y = coordinate.pose.position.y
            nodes.append((x, y))

        self.vectorizer.set_path(nodes)

        vectors = self.vectorizer.path_to_vectors()
        self.pub.publish(json.dumps(vectors))


def path_follower():
    rospy.init_node('path_following', anonymous=True)

    pathfollower = PathFollower()

    rate = rospy.Rate(10)  # 10hz

    rospy.spin()


if __name__ == '__main__':
    try:
        path_follower()
    except rospy.ROSInterruptException:
        pass
