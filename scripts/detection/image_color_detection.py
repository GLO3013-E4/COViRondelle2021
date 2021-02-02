import argparse

import cap
import cv2
import lower_color
import upper_color
import numpy as np
import color

ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", help="path to the image")
ap.add_argument("-c", "--color", help="color to detect")
args = vars(ap.parse_args())

image_to_detect = cv2.imread(args["image"])
color_to_detect = args["color"]


class ImageDetection:

    def __init__(self, image, color_to_detect):
        self.image = image
        self.color_to_detect = color_to_detect
        cv2.namedWindow('Color detection', cv2.WINDOW_NORMAL)

    def detect_image_color(self):
        red_lower = lower_color.get_red_lower()
        red_upper = upper_color.get_red_upper()

        green_lower = lower_color.get_green_lower()
        green_upper = upper_color.get_green_upper()

        if self.color_to_detect == color.Color.blue.value:
            self.detect_blue_color()

        elif self.color_to_detect == color.Color.red:
            pass

        elif self.color_to_detect == color.Color.green:
            pass
        else:
            pass

    def detect_blue_color(self):
        blue_lower = lower_color.get_blue_lower()
        blue_upper = upper_color.get_blue_upper()
        hsv_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        blue_mask = cv2.inRange(hsv_image, blue_lower, blue_upper)
        output_image = cv2.bitwise_and(self.image, self.image, mask=blue_mask)
        cv2.imshow("Color detection", np.hstack([output_image]))
        self.destroy_windows()

    def destroy_windows(self):
        while 1:
            if cv2.waitKey(10) & 0xFF == ord('q'):
                cap.release()
                cv2.destroyAllWindows()
                break


image_detection = ImageDetection(image_to_detect, color_to_detect)
image_detection.detect_image_color()
