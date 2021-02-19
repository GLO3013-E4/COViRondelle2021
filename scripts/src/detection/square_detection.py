import cv2

from object_detection import ObjectDetection

from scripts.src.detection.position import Position


class SquareDetection(ObjectDetection):

    def __init__(self, image):
        super().__init__(image, "square", 440, 550)
        self.minimum_area = 26000
        self.maximum_area = 28000

    def detect_square(self):
        cv2.namedWindow('Square detection', cv2.WINDOW_NORMAL)
        image_copy = self.copy_image()
        square_position = self._find_color(image_copy)
        self._show_image(image_copy)
        return square_position

    def _find_color(self, image_copy):
        image_hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)

        color_lower_boundary = self.lower_boundary.get_lower_boundaries(self.object_to_detect)
        color_upper_boundary = self.upper_boundary.get_upper_boundaries(self.object_to_detect)

        mask = cv2.inRange(image_hsv, color_lower_boundary, color_upper_boundary)
        return self._get_contours(mask, image_copy)

    def generate_four_corners(self, x_position, y_position, width, height, image_copy):
        corner_a = Position( x_position + width, y_position )
        corner_b = Position( x_position + width, y_position + height )
        corner_c = Position( x_position, y_position + height )
        corner_d = Position( x_position, y_position )

        cv2.putText( image_copy, "A", (corner_a.get_position_x(), corner_a.get_position_y()),
                     cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0), 2 )
        cv2.putText( image_copy, "B", (corner_b.get_position_x(), corner_b.get_position_y()),
                     cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0), 2 )
        cv2.putText( image_copy, "C", (corner_c.get_position_x(), corner_c.get_position_y()),
                     cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0), 2 )
        cv2.putText( image_copy, "D", (corner_d.get_position_x(), corner_d.get_position_y()),
                     cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0), 2 )

        four_corners = {
            "corner_A": corner_a,
            "corner_B": corner_b,
            "corner_C": corner_c,
            "corner_D": corner_d
        }
        return four_corners

    def _get_contours(self, image_mask, image_copy):
        contours, hierarchy = cv2.findContours(image_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        corner_position = self.generate_four_corners(0, 0, 0, 0, image_copy)

        for contour in contours:
            area = cv2.contourArea(contour)
            if self.is_in_area(area):
                perimeter = cv2.arcLength(contour, True)
                zone_approximation = cv2.approxPolyDP(contour, 0.05 * perimeter, True)
                object_corner = len( zone_approximation )
                x_position, y_position, width, height = cv2.boundingRect(zone_approximation)

                if self.object_is_in_range(width, height):
                    self.draw_rectangle_on_image( image_copy, x_position, y_position, width, height,
                                                  self.get_object_name( object_corner ) )

                    corner_position = self.generate_four_corners(x_position, y_position,
                                                                 width, height, image_copy)

        return corner_position


    def get_object_name(self, object_corner):
        if object_corner == 8:
            object_type = str(self.object_to_detect)
        else:
            object_type = "None"
        return object_type

    def is_in_area(self, area):
        return self.minimum_area < area < self.maximum_area


square_detection = SquareDetection("camera_monde_qr.jpg")
square_detection.detect_square()
