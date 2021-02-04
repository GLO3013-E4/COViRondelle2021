import argparse

import cap
import cv2
import lower_color
import upper_color
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", help="path to the image")
ap.add_argument("-c", "--color", help="color to detect")
args = vars(ap.parse_args())

image_to_detect = args["image"]
color_to_detect = args["color"]


class PuckDetection:

    def __init__(self, image, color_to_detect):
        self.image = cv2.imread(image)
        self.color_to_detect = color_to_detect
        cv2.namedWindow('Color detection', cv2.WINDOW_NORMAL)

    def detect_image_color(self):
        lower_boundary = lower_color.LowerBoundary()
        upper_boundary = upper_color.UpperBoundary()

        blue_lower = lower_boundary.get_lower(self.color_to_detect)
        blue_upper = upper_boundary.get_upper_boundary(self.color_to_detect)

        hsv_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        blue_mask = cv2.inRange(hsv_image, blue_lower, blue_upper)
        output_image = cv2.bitwise_and(self.image, self.image, mask=blue_mask)

        cv2.imshow("Color detection", np.hstack([output_image]))

        #cv2.waitKey(5000)

        # for infinite
        self.destroy_windows()

    def destroy_windows(self):
        while 1:
            if cv2.waitKey(10) & 0xFF == ord('q'):
                cap.release()
                cv2.destroyAllWindows()
                break


image_detection = PuckDetection(image_to_detect, color_to_detect)
image_detection.detect_image_color()
