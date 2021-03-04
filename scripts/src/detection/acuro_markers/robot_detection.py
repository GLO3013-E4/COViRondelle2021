import cv2

from scripts.src.detection.acuro_markers.AcuroMarkers import ArucoMarkers


class RobotDetection(ArucoMarkers):

    def detect_robot(self, image, DEBUG=True):
        image = self.capture_image_from_path(image)
        aruco_dict = self.get_acuro_dictionnary()
        aruco_params = self.get_acuro_params()

        robot_position = {}
        if image is None:
            return robot_position

        (corners, ids, rejected) = cv2.aruco.detectMarkers(image, aruco_dict,
                                       parameters=aruco_params)
        if len(corners) > 0:
            ids = ids.flatten()
            for (markerCorner, markerID) in zip(corners, ids):
                corners = markerCorner.reshape((4, 2))
                (top_left_position, top_right_position, bottom_right_position,
                 bottom_left_position) = corners

                bottom_left_position, bottom_right_position, top_left_position,\
                top_right_position = \
                    self.get_markers_corners_position(
                    bottom_left_position, bottom_right_position, top_left_position,
                        top_right_position)

                self.draw_line_on_markers(bottom_left_position, bottom_right_position,
                                          image,
                                          top_left_position,
                                          top_right_position)

                center_x, center_y = self.generate_center_position(
                    bottom_right_position=bottom_right_position,
                    top_left_position=top_left_position)

                self.generate_robot_position(bottom_left_position,
                                             bottom_right_position,
                                             center_x,
                                             center_y,
                                             robot_position,
                                             top_left_position,
                                             top_right_position)

                if DEBUG:
                    self.draw_center_position(center_x, center_y, image)
                    cv2.putText(image, str(markerID),
                            (top_left_position[0], top_left_position[1] - 15),
                                cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 255, 0), 2)
                print("[INFO] ArUco marker ID: {}".format(markerID))
            if DEBUG:
                self.show_image(image)

        print(robot_position)
        return robot_position

    def generate_robot_position(self, bottom_left_position, bottom_right_position,
                                center_x,
                                center_y,
                                robot_position,
                                top_left_position,
                                top_right_position):
        robot_position["center"] = (center_x, center_y)
        robot_position["top_right"] = top_right_position
        robot_position["top_left"] = top_left_position
        robot_position["bottom_right"] = bottom_right_position
        robot_position["bottom_left"] = bottom_left_position

    def get_acuro_dictionnary(self):
        return cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_50)
