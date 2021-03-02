#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from functools import partial
from scripts.src.pathfinding.show_path import get_path as get_nodes


class PathFinder:
    def __init__(self):
        self.goal = None
        self.pucks = None
        self.obstacles = None
        self.position = None
        self.node_size = 25
        self.algorithm = "BreadthFirstSearch"
        self.image_path = "bleh" # TODO:
        self.pathfinder = "bleh" # TODO:

        self.pub = rospy.Publisher('path', String, queue_size=10)
        rospy.Subscriber('goal', String, self.callback_goal)
        rospy.Subscriber('pucks', String, self.callback_pucks)
        rospy.Subscriber('obstacles', String, self.callback_obstacles)
        rospy.Subscriber('position', String, self.callback_position)
        rospy.Subscriber('get_path', String, self.get_path)

    def can_create_path(self):
        return self.goal and self.pucks and self.obstacles and self.position

    def get_path(self):
        return get_nodes(self.node_size, self.algorithm, self.obstacles, self.position, self.goal, self.pucks, self.image_path)

    def callback_goal(self, goal):
        self.pathfinder.goal = goal

    def callback_pucks(self, pucks):
        self.pathfinder.pucks = pucks

    def callback_obstacles(self, obstacles):
        self.pathfinder.obstacles = obstacles

    def callback_position(self, position):
        self.pathfinder.position = position


def get_path(self, pub, data):
    if self.pathfinder.can_create_path():
        # TODO: quoi faire si je pogne l'exception PathNotFound
        path = self.pathfinder.get_path()
        pub.publish(path)

        rospy.loginfo("Wooh j'ai posté le path")  # TODO: enlever le log
    else:
        rospy.loginfo("Manque des infos")  # TODO: changer le message


def path_planner():
    rospy.init_node('path_planning', anonymous=True)

    pub = rospy.Publisher('path', String, queue_size=10)

    pathfinder = PathFinder()

    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    try:
        path_planner()
    except rospy.ROSInterruptException:
        pass
