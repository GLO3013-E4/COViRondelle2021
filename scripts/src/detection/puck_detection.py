import cv2
import numpy as np

from scripts.src.detection.lower_boundary import LowerBoundary
from scripts.src.detection.upper_boundary import UpperBoundary


class PuckDetection:

    def __init__(self, image, color):
        self.lower_boundary = LowerBoundary()
        self.upper_boundary = UpperBoundary()
        self.puck_minimum_dimension = 35
        self.puck_maximum_dimension = 65
        self.minimum_area = 1440
        self.maximum_area = 2200
        self.image = cv2.imread(image)
        self.color_to_detect = color

    def detect_puck(self):
        cv2.namedWindow('Color detection', cv2.WINDOW_NORMAL)

        image_copy = self.image.copy()
        puck_position = self._find_color(image_copy)

        cv2.imshow("Color detection", np.hstack([image_copy]))
        cv2.waitKey(10000)

        return puck_position

    def _find_color(self, image_copy):
        image_hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)

        color_lower_boundary = self.lower_boundary.get_lower_boundaries(self.color_to_detect)
        color_upper_boundary = self.upper_boundary.get_upper_boundaries(self.color_to_detect)

        mask = cv2.inRange(image_hsv, color_lower_boundary, color_upper_boundary)
        return self._get_contours(mask, image_copy)

    def _get_contours(self, image_mask, image_copy):
        contours, hierarchy = cv2.findContours(image_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        for contour in contours:
            area = cv2.contourArea(contour)

            if self.is_in_area(area):
                perimeter = cv2.arcLength(contour, True)
                zone_approximation = cv2.approxPolyDP(contour, 0.05 * perimeter, True)
                object_corner = len(zone_approximation)
                x_position, y_position, width, height = cv2.boundingRect(zone_approximation)

                if self.object_is_in_range(width, height):
                    self.draw_rectangle_on_image(image_copy, x_position, y_position, width, height,
                                                 self.get_object_name(object_corner))
                    break
        try:
            puck_position = self.generate_puck_position(x_position, y_position, width, height)
        except NameError:
            puck_position = self.generate_puck_position(0, 0, 0, 0)
        return puck_position

    def draw_rectangle_on_image(self, image_copy, x_position, y_position, width, height, object_type):
        cv2.rectangle( image_copy, (x_position, y_position), (x_position + width, y_position + height), (0, 255, 0), 2 )
        cv2.putText( image_copy, object_type, (x_position + (width // 2) - 30, y_position + (height // 3) - 30),
                     cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0), 2 )

    def get_object_name(self, object_corner):
        if object_corner >= 4:
            object_type = str(self.color_to_detect) + " puck"
        else:
            object_type = "None"
        return object_type

    def object_is_in_range(self, width, height):
        return self.puck_minimum_dimension < width < self.puck_maximum_dimension and \
               self.puck_minimum_dimension < height < self.puck_maximum_dimension

    def generate_puck_position(self, x_position, y_position, width, height):
        return {
            "x_position": x_position,
            "y_position": y_position,
            "width": width,
            "height": height
        }

    def _destroy_windows(self):
        while 1:
            if cv2.waitKey(10) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
                break

    def is_in_area(self, area):
        return self.minimum_area < area < self.maximum_area


