import cv2
import numpy as np
from pyzbar.pyzbar import decode


class RobotDetection:

    def __init__(self, image):
        self.message = "robot"
        self.image = cv2.imread(image)
        self.robot = ""

    def detect_qr_code(self):
        for qr_code in decode(self.image):
            self.decode_message(qr_code)
            qr_codes_position = np.array([qr_code.polygon], np.int32)
            qr_codes_position = qr_codes_position.reshape((-1, 1, 2))
            cv2.polylines(self.image, [qr_codes_position], True, (255, 0, 255), 5)

            self.show_image()

    def decode_message(self, qr_code):
        message = qr_code.data.decode('utf-8')
        if message == "robot":
            self.message = "robot"
        elif message == "obstacle":
            self.message = "obstacle"

    def show_image(self):
        cv2.imshow("Qr Code", np.hstack([self.image]))
        cv2.waitKey(10000)

robot_detection = RobotDetection("qr_code_obstacle.png")
robot_detection.detect_qr_code()



