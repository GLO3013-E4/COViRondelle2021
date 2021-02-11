import cv2

from scripts.src.detection.lower_boundary import LowerBoundary
from scripts.src.detection.upper_boundary import UpperBoundary


class ObstacleDetection:
    def __init__(self, image):
        self.image = cv2.imread(image)
        self.lower_boundary = LowerBoundary()
        self.upper_boundary = UpperBoundary()
        self.minimum_area = 500
        self.maximum_area = 3000
        self.obstacle_side = 50


    def do_nothing(self):
        pass

    def detect_obstacle(self):
        return 2
