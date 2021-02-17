import cv2
import numpy as np
from pyzbar.pyzbar import decode


class QrDetection:

    def __init__(self, image):
        self.image = cv2.imread(image)

    def detect_qr_code(self, code_to_detect):
        if code_to_detect == "robot":
            return self.detect_robot()
        elif code_to_detect == "obstacle":
            return self.detect_obstacle()
        elif code_to_detect == "both":
            return self.detect_robot_and_obstacle()
        else:
            return 0

    def show_image(self):
        cv2.imshow("Qr Code", np.hstack([self.image]))
        cv2.waitKey(100000)

    def detect_robot(self):
        for qr_code in decode(self.image):
            message = qr_code.data.decode('utf-8')
            if message == "robot":
                qr_codes_position = np.array([qr_code.polygon], np.int32)
                qr_codes_position = qr_codes_position.reshape((-1, 1, 2))
                cv2.polylines(self.image, [qr_codes_position], True, (255, 0, 255), 5)
                self.show_image()
                return self.generate_qr_code_position(qr_code.polygon)
        return 0


    def generate_qr_code_position(self, qr_code_points):
        point_dictionary = {}
        for index, qr_point in enumerate(qr_code_points):
            point_dictionary[f"point {index + 1}"] = qr_point
        return point_dictionary



    def detect_robot_and_obstacle(self):
        pass


    def detect_obstacle(self):
        pass


qr_detection = QrDetection("monde_qr.jpg")
qr_detection.detect_qr_code("robot")


