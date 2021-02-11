import cv2
import numpy as np

from lower_boundary import LowerBoundary
from upper_boundary import UpperBoundary


class ObstacleDetection:
    def __init__(self, image):
        self.image = cv2.imread(image)
        self.lower_boundary = LowerBoundary()
        self.upper_boundary = UpperBoundary()
        self.minimum_area = 500
        self.maximum_area = 3000
        self.obstacle_side = 50
        self.name = "obstacle"

    def detect_obstacle(self):
        cv2.namedWindow('Color detection', cv2.WINDOW_NORMAL)

        image_copy = self.image.copy()
        mask = self.find_obstacle(image_copy)

        cv2.imshow("Color detection", np.hstack([mask]))
        cv2.waitKey(10000)
        return 2

    def find_obstacle(self, image_copy):
        image_hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)

        obstacle_lower_boundary = self.lower_boundary.get_lower_boundaries(self.name)
        obstacle_upper_boundary = self.upper_boundary.get_upper_boundaries(self.name)

        mask = cv2.inRange(image_hsv, obstacle_lower_boundary, obstacle_upper_boundary)
        return mask


obstacle_test = ObstacleDetection("monde3.jpg")
obstacle_test.detect_obstacle()
