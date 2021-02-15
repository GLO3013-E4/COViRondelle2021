import cv2
import numpy as np
from pyzbar.pyzbar import decode


class RobotDetection:

    def __init__(self, image):
        self.image = cv2.imread(image)

    def detect_qr_code(self):
        for qr_code in decode(self.image):
            qr_codes_position = np.array([qr_code.polygon], np.int32)
            qr_codes_position = qr_codes_position.reshape((-1, 1, 2))
            cv2.polylines(self.image, [qr_codes_position], True, (255, 0, 255), 5)

            self.show_image()

    def show_image(self):
        cv2.imshow("Qr Code", np.hstack([self.image]))
        cv2.waitKey(10000)

robot_detection = RobotDetection("qr_code_detection.png")
robot_detection.detect_qr_code()



