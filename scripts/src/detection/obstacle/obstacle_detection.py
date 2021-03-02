import cv2
import os

from obstacle_position import ObstaclePosition


class ObstacleDetection:

    def __init__(self):
        self.obstacle_position = ObstaclePosition()

    def capture_image_from_path(self, path):
        absolute_path = os.path.join(os.getcwd(), path)
        image = cv2.imread(absolute_path)

        if image is None:
            raise Exception(f'Image not found from path : {absolute_path}')

        return image

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


    def show_image(self, image):
        cv2.imshow("Markers", image)
        cv2.waitKey(10000)

    def draw_center_position(self, center_x, center_y, image):
        cv2.circle(image, (center_x, center_y), 4, (0, 0, 255), -1)

    def get_markers_corners_position(self, bottom_left_position, bottom_right_position, top_left_position,
                                     top_right_position):
        top_right_position = (int(top_right_position[0]), int(top_right_position[1]))
        bottom_right_position = (int(bottom_right_position[0]), int(bottom_right_position[1]))
        bottom_left_position = (int(bottom_left_position[0]), int(bottom_left_position[1]))
        top_left_position = (int(top_left_position[0]), int(top_left_position[1]))
        return bottom_left_position, bottom_right_position, top_left_position, top_right_position

    def draw_line_on_markers(self, bottom_left_position, bottom_right_position, image, top_left_position,
                             top_right_position):
        cv2.line(image, top_left_position, top_right_position, (0, 255, 0), 2)
        cv2.line(image, top_right_position, bottom_right_position, (0, 255, 0), 2)
        cv2.line(image, bottom_right_position, bottom_left_position, (0, 255, 0), 2)
        cv2.line(image, bottom_left_position, top_left_position, (0, 255, 0), 2)

    def get_acuro_params(self):
        return cv2.aruco.DetectorParameters_create()

    def detect_markers(self, aruco_dict, aruco_params, image_with_obstacle):
        return cv2.aruco.detectMarkers(image_with_obstacle, aruco_dict,
                                       parameters=aruco_params)

    def get_acuro_dictionnary(self):
        return cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
