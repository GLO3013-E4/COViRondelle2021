import cv2

from obstacle_position import ObstaclePosition
from AcuroMarkers import ArucoMarkers


class ObstacleDetection(ArucoMarkers):

    def __init__(self):
        self.obstacle_position = ObstaclePosition()

    def detect_obstacle(self, image):
        image = self.capture_image_from_path(image)
        aruco_dict = self.get_acuro_dictionnary()
        aruco_params = self.get_acuro_params()
        (corners, ids, rejected) = cv2.aruco.detectMarkers(image, aruco_dict,
                                       parameters=aruco_params)

        obstacles_position = []

        if len(corners) > 0:
            ids = ids.flatten()
            for (markerCorner, markerID) in zip(corners, ids):
                corners = markerCorner.reshape((4, 2))
                (top_left_position, top_right_position, bottom_right_position, bottom_left_position) = corners

                bottom_left_position, bottom_right_position, top_left_position, top_right_position = \
                    self.get_markers_corners_position(
                    bottom_left_position, bottom_right_position, top_left_position, top_right_position)

                self.draw_line_on_markers(bottom_left_position, bottom_right_position, image, top_left_position,
                                          top_right_position)

                obstacle_position = self.obstacle_position.generate_obstacle_dict(top_right=top_right_position,
                                                                                  top_left=top_left_position,
                                                                                  bottom_right=bottom_right_position,
                                                                                  bottom_left=bottom_left_position,
                                                                                  obstacle_id=str(markerID)
                                                                                  )
                obstacles_position.append(obstacle_position)
                center_x, center_y = self.obstacle_position.generate_center_position(bottom_right_position, top_left_position)

                self.draw_center_position(center_x, center_y, image)
                cv2.putText(image, str(markerID),
                            (top_left_position[0], top_left_position[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 255, 0), 2)
                print("[INFO] ArUco marker ID: {}".format(markerID))
            print(obstacles_position)
            self.show_image(image)
            return obstacles_position


    def get_acuro_dictionnary(self):
        return cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
