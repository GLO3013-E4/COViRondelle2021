#!/usr/bin/env python
from __future__ import division
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from pathfinding.show_path import get_path as get_nodes


class PathFinder:
    def __init__(self):
        self.goal = None
        self.pucks = None
        self.obstacles = None
        self.robot = None
        self.expanding_factor = 100
        self.node_size = 15
        self.algorithm = "BreadthFirstSearch"
        self.image_width, self.image_height = (640, 480)

        self.pub = rospy.Publisher('path', Path, queue_size=10)
        self.pub_string = rospy.Publisher('path_string', String, queue_size=10)
        rospy.Subscriber('goal', PoseStamped, self.callback_goal)
        rospy.Subscriber('pucks', PoseArray, self.callback_pucks)
        rospy.Subscriber('obstacles', PoseArray, self.callback_obstacles)
        rospy.Subscriber('robot', PoseStamped, self.callback_robot)

    def can_create_path(self):
        return self.goal and self.pucks and self.obstacles and self.robot

    def get_path(self):
        if self.can_create_path():
            nodes = get_nodes(self.node_size, self.algorithm, self.obstacles, self.robot, self.goal, self.pucks, self.image_width, self.image_height)

            path = Path()
            for node in nodes:
                pose = PoseStamped()
                pose.pose.position.x = node.pixel_coordinates_center[0]/ self.expanding_factor
                pose.pose.position.y = node.pixel_coordinates_center[1]/ self.expanding_factor
                pose.pose.position.z = 0
                pose.header.frame_id = "map"
                path.poses.append(pose)
                path.header.frame_id = "map"

            # pose = PoseStamped()
            # pose.pose.position.x = self.goal[0]/ self.expanding_factor
            # pose.pose.position.y = self.goal[1]/ self.expanding_factor
            # pose.pose.position.z = 0
            # pose.header.frame_id = "map"
            # path.poses.append(pose)
            # path.header.frame_id = "map"

            self.pub.publish(path)
        else:
            rospy.logerr("A parameter was still set to None (goal, pucks, obstacles, robot)")

    def callback_goal(self, goal):
        goal = goal.pose
        self.goal = (int(self.expanding_factor*goal.position.x), int(self.expanding_factor*goal.position.y))
        self.get_path()

    def callback_pucks(self, pucks):
        self.pucks = [(int(self.expanding_factor * puck.position.x), int(self.expanding_factor*puck.position.y)) for puck in pucks.poses]

    def callback_obstacles(self, obstacles):
        self.obstacles = [(int(self.expanding_factor * obstacle.position.x), int(self.expanding_factor*obstacle.position.y)) for obstacle in obstacles.poses]

    def callback_robot(self, robot):
        robot = robot.pose
        self.robot = (int(self.expanding_factor * robot.position.x), int(self.expanding_factor * robot.position.y))


def path_planner():
    rospy.init_node('path_planning', anonymous=True)

    pathfinder = PathFinder()

    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    try:
        path_planner()
    except rospy.ROSInterruptException:
        pass
