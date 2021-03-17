from typing import List

from obstacle_detection import ObstacleDetection
from marker_position import MarkerPosition

import numpy as np
import cv2
import os



class ObstacleRobotFinder:
    def __init__(self):
        self.obstacle_detection = ObstacleDetection()
        self.image_with = 1600 # mm
        self.image_height = 904 # mm
        self.obstacle_height = 412 #mm
        self.aruco_marker_width = 100 # mm
        self.obstacle_radius = 50
        self.distortion_coefficients  = np.array(
            [
                [
                    0.055764032942161694,
                    -0.1700050453380352,
                    -0.0028056916670508593,
                    0.0006434607299710345,
                    0.0331770702717552
                ]
        ])
        self.camera_matrix = np.array([
        [
            1321.5030177675765,
            0.0,
            763.385168511886
        ],
        [
            0.0,
            1327.9592573621323,
            494.93250836436187
        ],
        [
            0.0,
            0.0,
            1.0
        ]
    ])

    def detect_obstacle_position(self, image):
        script_dir = os.path.dirname(__file__)
        rel_path = image
        abs_file_path = os.path.join(script_dir, rel_path)
        image = cv2.imread(abs_file_path)

        obstacles_position = self.obstacle_detection.detect_obstacle(image, DEBUG=False)

        obstacles_3d_positions = self.obstacle_detection\
            .calculate_obstacle_position(obstacles_position=obstacles_position,
                                         aruco_marker_width=self.aruco_marker_width,
                                         camera_matrix=self.camera_matrix,
                                         distortion_coefficient=self.distortion_coefficients)

        for marker_position in obstacles_3d_positions:
            marker_position.set_markers_points(np.array([[0.0, 0.0, self.obstacle_height]]))
            marker_position.set_rotation_vector(np.array([[0.0, 0.0, 0.0]]))

        image_copy, center_of_bottom_obstacle = self.draw_obstacle_bottom(image=image, markers_position=obstacles_3d_positions)

        cv2.imshow("Est-ce que ca marche mon vieux max", image_copy)
        cv2.waitKey(0)

        return center_of_bottom_obstacle, self.obstacle_radius



    def draw_obstacle_bottom(self, markers_position: List[MarkerPosition], image):
        if image is None:
            return []

        image_copy = image.copy()
        obstacles_bottom_position = []
        for marker_position in markers_position:
            center_of_bottom_obstacle, _ = cv2.projectPoints(
                marker_position.get_markers_points(),
                marker_position.get_rotation_vector(),
                marker_position.get_translation_vector(),
                self.camera_matrix,
                self.distortion_coefficients
            )

            center_of_bottom_obstacle = tuple(center_of_bottom_obstacle.reshape(2, ).astype(np.int32))
            obstacles_bottom_position.append(center_of_bottom_obstacle)
            image_copy = cv2.circle(image_copy, center_of_bottom_obstacle, self.obstacle_radius, (0, 255, 255), 2)

        return image_copy, obstacles_bottom_position



AN_IMAGE = "image_obstacle.jpeg"
obstacle_robot_finder = ObstacleRobotFinder()
x, r = obstacle_robot_finder.detect_obstacle_position(image=AN_IMAGE)
print(x, r)
