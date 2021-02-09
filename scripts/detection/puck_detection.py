import cap
import cv2
from lower_color import LowerBoundary
from upper_color import UpperBoundary
import numpy as np
import argparse


class PuckDetection:

    def __init__(self, image, color):
        self.lower_boundary = LowerBoundary()
        self.upper_boundary = UpperBoundary()
        self.image = cv2.imread(image)
        self.color_to_detect = color

    def detect_image_color(self):
        cv2.namedWindow('Color detection', cv2.WINDOW_NORMAL)

        image_copy = self.image.copy()
        self._find_color(image_copy)

        cv2.imshow("Color detection", np.hstack([image_copy]))
        self._destroy_windows()

    def _find_color(self, image_copy):
        image_hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)

        color_lower_boundary = self.lower_boundary.get_lower(self.color_to_detect)
        color_upper_boundary = self.upper_boundary.get_upper_boundary(self.color_to_detect)

        mask = cv2.inRange(image_hsv, color_lower_boundary, color_upper_boundary)
        self._get_contours(mask, image_copy)

    def _get_contours(self, image_mask, image_copy):
        contours, hierarchy = cv2.findContours(image_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        for contour in contours:
            area = cv2.contourArea(contour)

            if 600 < area < 2500:
                perimeter = cv2.arcLength(contour, True)
                zone_approximation = cv2.approxPolyDP(contour, 0.05 * perimeter, True)
                object_corner = len(zone_approximation)
                x, y, width, height = cv2.boundingRect(zone_approximation)
                if object_corner >= 4:
                    object_type = "Rondelle " + str(self.color_to_detect)

                else:
                    object_type = "None"

                cv2.rectangle(image_copy, (x, y), (x + width, y + height), (0, 255, 0), 2)
                cv2.putText(image_copy, object_type, (x + (width // 2) - 30, y + (height // 3) - 30),
                            cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0), 2)

    def _destroy_windows(self):
        while 1:
            if cv2.waitKey(10) & 0xFF == ord('q'):
                cap.release()
                cv2.destroyAllWindows()
                break


ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", help="path to the image")
ap.add_argument("-c", "--color", help="color to detect")
args = vars(ap.parse_args())

image_to_detect = args["image"]
color_to_detect = args["color"]

image_detection = PuckDetection(image_to_detect, color_to_detect)
image_detection.detect_image_color()
