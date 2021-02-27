import cv2
import numpy as np
from pyzbar.pyzbar import decode

from detection.qr_code_type import QrCodeTypes
from detection.utils.point import Point


class QrDetection:

    def __init__(self, image):
        self.image = image

    def detect_qr_code(self, code_to_detect):
        if self.image is None:
            return self.generate_empty_qr_code_position()
        elif code_to_detect == QrCodeTypes.ROBOT.value:
            return self.detect_robot()
        elif code_to_detect == QrCodeTypes.OBSTACLE.value:
            return self.detect_obstacle()
        elif code_to_detect == QrCodeTypes.ROBOT_AND_OBSTACLE.value:
            return self.detect_robot_and_obstacle()
        else:
            return self.generate_empty_qr_code_position()

    def detect_robot(self):
        robot_position = self.generate_empty_qr_code_position()
        if self.image is not None:
            for qr_code in decode(self.image):
                message = qr_code.data.decode('utf-8')
                if message == QrCodeTypes.ROBOT.value:
                    robot_position = self._generate_qr_code_position(qr_code.polygon)
        return robot_position

    def detect_obstacle(self):
        obstacles_position = []
        if self.image is not None:
            for qr_code in decode(self.image):
                message = qr_code.data.decode('utf-8')
                if message == QrCodeTypes.OBSTACLE.value:
                    obstacles_position.append(self._generate_qr_code_position(
                        qr_code.polygon))
        self.add_empty_position_if_not_all_obstacle_found(obstacles_position)
        return obstacles_position

    def add_empty_position_if_not_all_obstacle_found(self, obstacles_position):
        empty_position = self.generate_empty_qr_code_position()
        if len(obstacles_position) == 1:
            obstacles_position.append(empty_position)
        if len(obstacles_position) == 0:
            obstacles_position.append(empty_position)
            obstacles_position.append(empty_position)
        return obstacles_position

    def detect_robot_and_obstacle(self):
        objects_position = {
            "obstacles": [],
            "robot": []
        }
        if self.image is not None:
            for qr_code in decode(self.image):
                message = qr_code.data.decode('utf-8')
                if message == QrCodeTypes.OBSTACLE.value:
                    objects_position["obstacles"].append(
                        self._generate_qr_code_position(qr_code.polygon))
                elif message == QrCodeTypes.ROBOT.value:
                    objects_position["robot"].append(
                        self._generate_qr_code_position(qr_code.polygon))
        self.add_empty_position_if_not_all_obstacle_found(objects_position["obstacles"])
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
            qr_position_point = Point(qr_point.x, qr_point.y).get_coordinates()
            point_key = "point" + str(index+1)
            point_dictionary[point_key] = qr_position_point
        return point_dictionary

    def generate_empty_qr_code_position(self):
        point_dictionary = {}
        point_with_position_of_zero = Point(0, 0).get_coordinates()
        for index in range(0, 4):
            point_key = "point" + str(index+1)
            point_dictionary[point_key] = point_with_position_of_zero
        return point_dictionary
