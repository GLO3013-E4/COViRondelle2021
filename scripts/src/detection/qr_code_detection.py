import cv2
import numpy as np
from pyzbar.pyzbar import decode


class QrDetection:

    def __init__(self, image):
        self.image = cv2.imread(image)

    def detect_qr_code(self, code_to_detect):
        if code_to_detect == "robot":
            self.detect_robot()
        elif code_to_detect == "obstacle":
            self.detect_obstacle()
        elif code_to_detect == "both":
            self.detect_robot_and_obstacle()
        else:
            return 0

    def show_image(self):
        cv2.imshow("Qr Code", np.hstack([self.image]))
        cv2.waitKey(10000)

    def detect_robot(self):
        for qr_code in decode(self.image):
            message = qr_code.data.decode('utf-8')
            qr_codes_position = np.array([qr_code.polygon], np.int32)
            qr_codes_position = qr_codes_position.reshape((-1, 1, 2))
            cv2.polylines(self.image, [qr_codes_position], True, (255, 0, 255), 5)
            self.show_image()

    def detect_robot_and_obstacle(self):
        pass

    def detect_obstacle(self):
        pass

