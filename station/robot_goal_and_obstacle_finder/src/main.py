#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from detection.qr_code_detection import QrDetection
from detection.puck_detection import PuckDetection
from detection.square_detection import SquareDetection
from detection.color import Color


class RobotGoalAndObstacleFinder:
    def __init__(self):
        self.robot_publisher = rospy.Publisher('robot', String, queue_size=10)
        self.pucks_publisher = rospy.Publisher('pucks', String, queue_size=10)
        self.obstacles_publisher = rospy.Publisher('obstacles', String, queue_size=10)
        self.square_publisher = rospy.Publisher('square', String, queue_size=10)
        self.resistance_station_publisher = rospy.Publisher('resistance_station', Pose, queue_size=10)
        self.image_subscriber = rospy.Subscriber('world_cam/image_raw', Image, self.callback)
        self.bridge = CvBridge()
        self.rate = rospy.Rate(1)

    def find_robot_and_obstacles(self, image):
        robot_finder = QrDetection(image)
        return robot_finder.detect_robot_and_obstacle()

    def find_puck(self, image):
        return PuckDetection(image, "red").detect_puck()
    
    def find_square(self, image):
        return SquareDetection(image).detect_square()
            
    def callback(self, data):
        # faire les calculs ici

        cv2_array = self.bridge.imgmsg_to_cv2(data, "rgb8")

        robot_and_obstacles = self.find_robot_and_obstacles(cv2_array)
        robot = robot_and_obstacles["robot"]
        obstacles = robot_and_obstacles["obstacles"]
        rospy.loginfo("sending sauce")

        square = self.find_square(cv2_array)
        red_puck = self.find_puck(cv2_array)

        self.robot_publisher.publish(str(robot))
        self.obstacles_publisher.publish(str(obstacles))
        self.square_publisher.publish(str(square))
        self.pucks_publisher.publish(str(red_puck))
        # self.robot_publisher 
        # self.pucks_publisher 
        # self.obstacles_publisher 
        # self.square_publisher 
        # self.resistance_station_publisher 

if __name__ == '__main__':
    try:
        rospy.init_node('reprojection_node', anonymous=True)
        robot_goal_and_obstacle_finder = RobotGoalAndObstacleFinder()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass