import cv2

from scripts.src.detection.object_detection import ObjectDetection


class ObstacleDetection(ObjectDetection):
    def __init__(self, image):
        super().__init__(image, "obstacle", 60, 120)

    def detect_obstacle(self):
        cv2.namedWindow('Color detection', cv2.WINDOW_NORMAL)
        image_copy = self.copy_image()
        obstacle_position = self.find_obstacle(image_copy)
        self.show_image(image_copy)
        return obstacle_position

    def find_obstacle(self, image_copy):
        image_hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)

        obstacle_lower_boundary = self.lower_boundary.get_lower_boundaries(self.name)
        obstacle_upper_boundary = self.upper_boundary.get_upper_boundaries(self.name)

        mask = cv2.inRange(image_hsv, obstacle_lower_boundary, obstacle_upper_boundary)
        return self.get_contours(mask, image_copy)

    def get_contours(self, mask, image_copy):
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        obstacles_position = []

        for contour in contours:

            perimeter = cv2.arcLength(contour, True)
            zone_approximation = cv2.approxPolyDP(contour, 0.05 * perimeter, True)
            x_position, y_position, width, height = cv2.boundingRect(zone_approximation)

            if self.object_is_in_range(width, height):
                self.draw_rectangle_on_image(image_copy, x_position, y_position, width, height, self.name)
                obstacles_position.append(self.generate_puck_position(x_position, y_position, width, height))

        return obstacles_position