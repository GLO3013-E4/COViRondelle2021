import cv2

from lower_boundary import LowerBoundary
from upper_boundary import UpperBoundary


class ObjectDetection:

    def __init__(self, image):
        self.image = cv2.imread(image)
        self.lower_boundary = LowerBoundary()
        self.upper_boundary = UpperBoundary()

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