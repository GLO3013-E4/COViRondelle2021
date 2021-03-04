import cv2

from scripts.src.detection.object_detection import ObjectDetection


class PuckDetection(ObjectDetection):

    def __init__(self, image, color):
        super().__init__(image, color, 35, 65)
        self.minimum_area = 1440
        self.maximum_area = 2200

    def detect_puck(self):
        cv2.namedWindow('Color detection', cv2.WINDOW_NORMAL)
        image_copy = self.copy_image()
        puck_position = self._find_color(image_copy)
        self._show_image(image_copy)
        return puck_position

    def _find_color(self, image_copy):
        image_hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)

        color_lower_boundary = self.lower_boundary.get_lower_boundaries( self.object_to_detect )
        color_upper_boundary = self.upper_boundary.get_upper_boundaries( self.object_to_detect )

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
                    try:
                        puck_position = self.generate_puck_position(x_position, y_position,
                                                                    width, height)
                    except NameError:
                        puck_position = self.generate_puck_position(0, 0, 0, 0)
                    return puck_position
        return 0

    def get_object_name(self, object_corner):
        if object_corner >= 4:
            object_type = str( self.object_to_detect ) + " puck"
        else:
            object_type = "None"
        return object_type

    def is_in_area(self, area):
        return self.minimum_area < area < self.maximum_area
