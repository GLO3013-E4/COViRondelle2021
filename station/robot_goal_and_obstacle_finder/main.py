#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class RobotGoalAndObstacleFinder:
    def __init__(self):
        self.robot_publisher = rospy.Publisher('robot', String, queue_size=10)
        self.pucks_publisher = rospy.Publisher('pucks', String, queue_size=10)
        self.obstacles_publisher = rospy.Publisher('obstacles', String, queue_size=10)
        self.square_publisher = rospy.Publisher('square', String, queue_size=10)
        self.resistance_station_publisher = rospy.Publisher('resistance_station', Pose, queue_size=10)
        self.image_subscriber = rospy.Subscriber('world_cam/image_raw', Image, self.detect)
        self.bridge = CvBridge()
        self.rate = rospy.Rate(1)

    # 1 publisher par couleur?
    def find_puck(self, image):
        return

    def find_square(self, image):
        return

    def detect(self, world_camera_driver_image):
        cv2_array = self.bridge.imgmsg_to_cv2(world_camera_driver_image, "rgb8")


        # self.robot_publisher.publish(str(robot))
        # self.obstacles_publisher.publish(str(obstacles))
        # self.square_publisher.publish(str(square))
        # self.pucks_publisher.publish(str(red_puck))
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
