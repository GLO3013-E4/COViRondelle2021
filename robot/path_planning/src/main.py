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

    def can_create_path(self):
        return self.goal and self.pucks and self.obstacles and self.position

    def get_path(self):
        return get_nodes(self.node_size, self.algorithm, self.obstacles, self.position, self.goal, self.pucks, self.image_path)


def callback_goal(path_finder, goal):
    path_finder.goal = goal


def callback_pucks(path_finder, pucks):
    path_finder.pucks = pucks


def callback_obstacles(path_finder, obstacles):
    path_finder.obstacles = obstacles


def callback_position(path_finder, position):
    path_finder.position = position


def get_path(path_finder, pub, data):
    if path_finder.can_create_path():
        # TODO: quoi faire si je pogne l'exception PathNotFound
        path = path_finder.get_path()
        pub.publish(path)

        rospy.loginfo("Wooh j'ai posté le path")  # TODO: enlever le log
    else:
        rospy.loginfo("Manque des infos")  # TODO: changer le message


def path_planner():
    rospy.init_node('path_planning', anonymous=True)

    pub = rospy.Publisher('path', String, queue_size=10)

    path_finder = PathFinder()

    rospy.Subscriber('goal', String, partial(callback_goal,path_finder=path_finder))
    rospy.Subscriber('pucks', String, partial(callback_pucks, path_finder=path_finder))
    rospy.Subscriber('obstacles', String, partial(callback_obstacles, path_finder=path_finder))
    rospy.Subscriber('position', String, partial(callback_position, path_finder=path_finder))
    rospy.Subscriber('get_path', String, partial(get_path, path_finder=path_finder, pub=pub))

    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    try:
        path_planner()
    except rospy.ROSInterruptException:
        pass
