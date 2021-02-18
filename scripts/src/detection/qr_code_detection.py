import cv2
import numpy as np
from pyzbar.pyzbar import decode

from scripts.src.detection.qr_code_type import QrCodeTypes


class QrDetection:

    def __init__(self, image):
        self.image = cv2.imread(image)

    def detect_qr_code(self, code_to_detect):
        try:
            if code_to_detect == QrCodeTypes.ROBOT.value:
                return self._detect_robot()
            elif code_to_detect == QrCodeTypes.OBSTACLE.value:
                return self._detect_obstacle()
            elif code_to_detect == QrCodeTypes.ROBOT_AND_OBSTACLE.value:
                return self._detect_robot_and_obstacle()
            else:
                return 0
        except TypeError:
            raise TypeError("Image invalide")


    def _detect_robot(self):
        for qr_code in decode(self.image):
            message = qr_code.data.decode('utf-8')
            if message == QrCodeTypes.ROBOT.value:
                return self._generate_qr_code_position(qr_code.polygon)
        return 0

    def _detect_obstacle(self):
        obstacles_position = []
        for qr_code in decode(self.image):
            message = qr_code.data.decode('utf-8')
            if message == QrCodeTypes.OBSTACLE.value:
                obstacles_position.append(self._generate_qr_code_position(qr_code.polygon))
        return obstacles_position

    def _detect_robot_and_obstacle(self):
        objects_position = {
            "obstacles": [],
            "robot": []
        }
        for qr_code in decode(self.image):
            message = qr_code.data.decode('utf-8')
            if message == QrCodeTypes.OBSTACLE.value:
                objects_position["obstacles"].append(self._generate_qr_code_position(qr_code.polygon))
            elif message == QrCodeTypes.ROBOT.value:
                objects_position["robot"].append(self._generate_qr_code_position(qr_code.polygon))
        return objects_position

    def _show_image(self):
        cv2.imshow("Qr Code", np.hstack([self.image]))
        cv2.waitKey(10000)

    def _draw_on_image(self, qr_code):
        qr_codes_position = np.array([qr_code.polygon], np.int32)
        qr_codes_position = qr_codes_position.reshape((-1, 1, 2))
        cv2.polylines(self.image, [qr_codes_position], True, (255, 0, 255), 5)

    def _generate_qr_code_position(self, qr_code_points):
        point_dictionary = {}
        for index, qr_point in enumerate(qr_code_points):
            point_dictionary[f"point {index + 1}"] = qr_point
        return point_dictionary