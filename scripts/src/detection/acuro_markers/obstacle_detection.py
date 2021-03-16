from typing import List

import cv2

from AcuroMarkers import ArucoMarkers
from obstacle_position import ObstaclePosition


class ObstacleDetection(ArucoMarkers):

    def detect_obstacle(self, image, DEBUG=True):
        image = self.capture_image_from_path(image)
        aruco_dict = self.get_acuro_dictionnary()
        aruco_params = self.get_acuro_params()

        print(image.shape)


        if image is None:
            return self.generate_empty_obstacle_position()

        (corners, ids, rejected) = cv2.aruco.detectMarkers(image, aruco_dict,
                                       parameters=aruco_params)
        obstacles_position = []


        if len(corners) > 0:
            ids = ids.flatten()
            for (markerCorner, markerID) in zip(corners, ids):
                obstacles_position.append(ObstaclePosition(markerID, markerCorner))
                corners = markerCorner.reshape((4, 2))
                (top_left_position, top_right_position, bottom_right_position,
                 bottom_left_position) = corners

                bottom_left_position, bottom_right_position, top_left_position, \
                top_right_position = \
                    self.get_markers_corners_position(
                    bottom_left_position, bottom_right_position, top_left_position,
                        top_right_position)

                self.draw_line_on_markers(bottom_left_position, bottom_right_position,
                                          image, top_left_position,
                                          top_right_position)


                center_x, center_y = self.generate_center_position(bottom_right_position,
                                                                   top_left_position)


                if DEBUG:
                    self.draw_center_position(center_x, center_y, image)
                    cv2.putText(image, str(markerID),
                            (top_left_position[0], top_left_position[1] - 15),
                                cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 255, 0), 2)
                print("[INFO] ArUco marker ID: {}".format(markerID))
            print(obstacles_position)

            if DEBUG:
                self.show_image(image)
        return obstacles_position


    def get_acuro_dictionnary(self):
        return cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

    def generate_empty_obstacle_position(self):
        obstacle_position = []
        for i in range(0, 2):
            obstacle_position.append({f"obstacle {i + 1}": {
                "center": (0, 0),
                "top_right": (0, 0),
                "top_left": (0, 0),
                "bottom_right": (0, 0),
                "bottom_left": (0, 0)
            }})
        return obstacle_position

    def calculate_3D_position(self, obstacles_position: List[ObstaclePosition],
                              aruco_marker_width,
                              camera_matrix,
                              distortion_coefficient):

        aruco_markers_corner = [obstacle_position.get_corner()
                                for obstacle_position in
                                obstacles_position]

        corner_length = len(aruco_markers_corner)

        aruco_markers_position = []
        if 1 > corner_length:
            return aruco_markers_position

        rotation_vectors, translation_vectors, objects_points = cv2.aruco.estimatePoseSingleMarkers(
            aruco_markers_corner,
            aruco_marker_width,
            camera_matrix,
            distortion_coefficient
        )

        aruco_markers_position = [
            MarkerPosition()
        ]




obstacle = ObstacleDetection()
obstacle.detect_obstacle("monde10.jpg")