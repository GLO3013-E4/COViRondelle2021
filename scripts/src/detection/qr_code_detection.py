import cv2
import numpy as np
from pyzbar.pyzbar import decode


class QrDetection:

    def __init__(self, image):
        self.image = cv2.imread(image)

    def detect_qr_code(self, code_to_detect):
        if code_to_detect == "robot":
            return self._detect_robot()
        elif code_to_detect == "obstacle":
            return self._detect_obstacle()
        elif code_to_detect == "both":
            return self._detect_robot_and_obstacle()
        else:
            return 0


    def _detect_robot(self):
        for qr_code in decode(self.image):
            message = qr_code.data.decode('utf-8')
            if message == "robot":
                return self._generate_qr_code_position(qr_code.polygon)
        return 0

    def _detect_obstacle(self):
        obstacles_position = []
        for qr_code in decode(self.image):
            message = qr_code.data.decode('utf-8')
            if message == "obstacle":
                single_obstacle =  self._generate_qr_code_position(qr_code.polygon)
                obstacles_position.append(single_obstacle)
        return obstacles_position

    def _detect_robot_and_obstacle(self):
        object_position = {
            "obstacles": []
        }
        for qr_code in decode(self.image):
            message = qr_code.data.decode('utf-8')
            if message == "obstacle":
                single_obstacle = self._generate_qr_code_position(qr_code.polygon)
                object_position["obstacles"].append(single_obstacle)
            elif message == "robot":
                object_position["robot_position"] = self._generate_qr_code_position(qr_code.polygon)
        return object_position

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


qr_detection = QrDetection("camera_monde2.jpg")
qr_detection.detect_qr_code("both")


