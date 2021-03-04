#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from path_planning.src.pathfinding.show_path import get_path as get_nodes


class PathFinder:
    def __init__(self):
        self.goal = None
        self.pucks = None
        self.obstacles = None
        self.robot = None
        self.node_size = 15
        self.algorithm = "BreadthFirstSearch"
        self.image_width, self.image_height = (640, 480)

        self.pub = rospy.Publisher('path', Path, queue_size=10)
        self.pub_string = rospy.Publisher('path_string', String, queue_size=10)
        rospy.Subscriber('goal', Pose, self.callback_goal)
        rospy.Subscriber('pucks', PoseArray, self.callback_pucks)
        rospy.Subscriber('obstacles', PoseArray, self.callback_obstacles)
        rospy.Subscriber('robot', Pose, self.callback_robot)

    def can_create_path(self):
        return self.goal and self.pucks and self.obstacles and self.robot

    def get_path(self):
        if self.can_create_path():
            nodes = get_nodes(self.node_size, self.algorithm, self.obstacles, self.robot, self.goal, self.pucks, self.image_width, self.image_height)

            path = Path()
            arr = []
            for node in nodes:
                arr.append(tuple(node.pixel_coordinates_center))
                pose = PoseStamped()
                pose.pose.position.x = node.pixel_coordinates_center[0]
                pose.pose.position.y = node.pixel_coordinates_center[1]
                pose.pose.position.z = 0
                pose.header.frame_id = "/map"
                path.poses.append(pose)
                path.header.frame_id = "/map"

            self.pub_string.publish(str(arr))
            self.pub.publish(path)
        else:
            rospy.logerr("A parameter was still set to None (goal, pucks, obstacles, robot)")

    def callback_goal(self, goal):
        self.goal = (int(goal.position.x), int(goal.position.y))
        self.get_path()

    def callback_pucks(self, pucks):
        self.pucks = [(int(puck.position.x), int(puck.position.y)) for puck in pucks.poses]

    def callback_obstacles(self, obstacles):
        self.obstacles = [(int(obstacle.position.x), int(obstacle.position.y)) for obstacle in obstacles.poses]

    def callback_robot(self, robot):
        self.robot = (int(robot.position.x), int(robot.position.y))


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
