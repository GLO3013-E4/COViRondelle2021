import cv2
import numpy as np

class ObstacleDetection:

    def __init__(self, image):
        self.image = cv2.imread(image)
        self.minimum_radius = 40
        self.maximum_radius = 60
        self.param_one = 100
        self.param_two = 60

    def detect_obstacle(self):
        image_blur, image_copy = self.get_blur_image()

        row = self.get_row_lenght(image_blur)
        circles = self.circle_detection(image_blur, row)

        obstacle_position = []

        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for (x_position, y_position, radius) in circles:
                obstacle_position.append(self.obstacle_position(x_position, y_position, radius))

        return obstacle_position


    def draw_color_on_circles(self, image_copy, radius, x_position, y_position):
        cv2.circle(image_copy, (x_position, y_position), radius, (0, 255, 0), 4)
        cv2.rectangle(image_copy, (x_position + 2, y_position + 2),
                      (x_position - 2, y_position- 2), (0, 128, 255), -1)

    def obstacle_position(self, x_position, y_position, radius):
        return {
            "x_position": x_position,
            "y_position": y_position,
            "radius": radius
        }

    def show_image(self, image_copy):
        cv2.imshow("output", np.hstack([image_copy]))
        cv2.waitKey(0)

    def get_row_lenght(self, image_blur):
        return image_blur.shape[0] / 8

    def circle_detection(self, image_blur, row):
        return cv2.HoughCircles(image_blur, cv2.HOUGH_GRADIENT, 1.1, row, param1=self.param_one,
                                param2=self.param_two,
                                minRadius=self.minimum_radius, maxRadius=self.maximum_radius)

    def get_blur_image(self):
        image_copy = self.image
        image_gray = cv2.cvtColor(image_copy, cv2.COLOR_BGR2GRAY)
        image_blur = cv2.GaussianBlur(image_gray, (7, 7), 1)
        return image_blur, image_copy
