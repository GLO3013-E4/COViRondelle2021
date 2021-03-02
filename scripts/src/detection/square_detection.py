import cv2

from scripts.src.detection.object_detection import ObjectDetection
from scripts.src.detection.utils.point import Point


class SquareDetection(ObjectDetection):

    def __init__(self, image):
        super().__init__(image, "square", 440, 550)
        self.minimum_area = 26000
        self.maximum_area = 28000

    def detect_square(self):
        return self._find_color()

    def _find_color(self):
        image_hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)

        color_lower_boundary = self.lower_boundary.get_lower_boundaries(self.object_to_detect)
        color_upper_boundary = self.upper_boundary.get_upper_boundaries(self.object_to_detect)

        mask = cv2.inRange(image_hsv, color_lower_boundary, color_upper_boundary)
        return self._get_contours(mask)

    def generate_four_corners(self, x_position, y_position, width, height):
        corner_a = Point(x_position + width, y_position)
        corner_b = Point(x_position + width, y_position + height)
        corner_c = Point(x_position, y_position + height)
        corner_d = Point(x_position, y_position)

        four_corners = {
            "corner_A": corner_a,
            "corner_B": corner_b,
            "corner_C": corner_c,
            "corner_D": corner_d
        }
        return four_corners

    def _get_contours(self, image_mask):
        contours, hierarchy = cv2.findContours(image_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        corner_position = self.generate_four_corners(0, 0, 0, 0)

        for contour in contours:
            area = cv2.contourArea(contour)
            if self.is_in_area(area):
                perimeter = cv2.arcLength(contour, True)
                zone_approximation = cv2.approxPolyDP(contour, 0.05 * perimeter, True)
                x_position, y_position, width, height = cv2.boundingRect(zone_approximation)

                if self.object_is_in_range(width, height):
                    corner_position = self.generate_four_corners(x_position, y_position,
                                                                 width, height)

        return corner_position


    def get_object_name(self, object_corner):
        if object_corner == 8:
            object_type = str(self.object_to_detect)
        else:
            object_type = "None"
        return object_type

    def is_in_area(self, area):
        return self.minimum_area < area < self.maximum_area


def detect_square(image):
    pass
   
detect_square("monde2.jpg")




