import cv2
import numpy as np

from lower_boundary import LowerBoundary
from upper_boundary import UpperBoundary


class ObstacleDetection:
    def __init__(self, image):
        self.image = cv2.imread(image)
        self.lower_boundary = LowerBoundary()
        self.upper_boundary = UpperBoundary()
        self.obstacle_minimum_dimension = 20
        self.obstacle_maximum_dimension = 300
        self.minimum_area = 2
        self.maximum_area = 3000
        self.obstacle_side = 50
        self.name = "obstacle"

    def detect_obstacle(self):
        cv2.namedWindow('Color detection', cv2.WINDOW_NORMAL)

        image_copy = self.image.copy()
        mask = self.find_obstacle(image_copy)

        cv2.imshow("Color detection", np.hstack([image_copy]))
        cv2.waitKey(100000)
        return 2

    def find_obstacle(self, image_copy):
        image_hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)

        obstacle_lower_boundary = self.lower_boundary.get_lower_boundaries(self.name)
        obstacle_upper_boundary = self.upper_boundary.get_upper_boundaries(self.name)

        mask = cv2.inRange(image_hsv, obstacle_lower_boundary, obstacle_upper_boundary)
        return self.get_contours(mask, image_copy)

    def get_contours(self, mask, image_copy):
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        for contour in contours:

            perimeter = cv2.arcLength(contour, True)
            zone_approximation = cv2.approxPolyDP(contour, 0.05 * perimeter, True)
            object_corner = len(zone_approximation)
            print(object_corner)
            x_position, y_position, width, height = cv2.boundingRect(zone_approximation)

            if self.object_is_in_range(width, height):
                self.draw_contours(x_position, y_position, width, height, image_copy)



    def object_is_in_range(self, width, height):
        return self.obstacle_minimum_dimension < width < self.obstacle_maximum_dimension and\
               self.obstacle_minimum_dimension < height < self.obstacle_maximum_dimension


    def draw_contours(self, x_position, y_position, width, height, image_copy):
        cv2.rectangle(image_copy, (x_position, y_position), (x_position + width, y_position + height), (0, 255, 0), 2)
        cv2.putText(image_copy, self.name, (x_position + (width // 2) - 30, y_position + (height // 3) - 30),
                    cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0), 2)


obstacle_test = ObstacleDetection("monde3.jpg")
obstacle_test.detect_obstacle()
