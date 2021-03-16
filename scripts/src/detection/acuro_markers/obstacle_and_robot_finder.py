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
        self.obstacle_pixel = 46
        self.distortion_coefficients  = [
        [
            4.767177288453392,
            0.016944982578597136,
            0.009067230348369367,
            0.03617489894832692,
            8.084050600445835e-06
        ]
    ]
        self.camera_matrix = [
        [
            20870.2343261594,
            0.0,
            799.4716877771021
        ],
        [
            0.0,
            20966.11071161727,
            452.15228742450336
        ],
        [
            0.0,
            0.0,
            1.0
        ]
    ]

    def detect_obstacle_position(self, image):
        script_dir = os.path.dirname(__file__)
        rel_path = image
        abs_file_path = os.path.join(script_dir, rel_path)
        image = cv2.imread(abs_file_path)

        obstacles_position = self.obstacle_detection.detect_obstacle(image)

        obstacles_3d_positions = self.obstacle_detection\
            .calculate_3D_position(obstacles_position=obstacles_position,
                                   aruco_marker_width=self.aruco_marker_width,
                                   camera_matrix=self.camera_matrix,
                                   distortion_coefficient=self.distortion_coefficients)

        for marker_position in obstacles_3d_positions:
            marker_position.set_markers_points(np.array([[0.0, 0.0, self.obstacle_height]]))
            marker_position.set_rotation_vector(np.array([[0.0, 0.0, 0.0, 0.0]]))

        image_copy = self.draw_obstacle_bottom(image=image, markers_position=obstacles_3d_positions)

        cv2.imshow("Est-ce que ca marche mon vieux max", image_copy)
        cv2.waitKey(0)



    def draw_obstacle_bottom(self, markers_position: List[MarkerPosition], image):
        print(markers_position)

        if image is None:
            return []

        image_copy = image.copy()

        for marker_position in markers_position:
            image_to_draw, _ = cv2.projectPoints(
                marker_position.get_markers_points(),
                marker_position.get_rotation_vector(),
                marker_position.get_translation_vector(),
                self.camera_matrix,
                self.distortion_coefficients
            )

            image_to_draw = tuple(image_to_draw.reshape(2, ).astype(np.int32))
            image_copy = cv2.circle(image_copy, image_to_draw, self.obstacle_pixel ,(0, 255, 255), 2)

        return image_copy



AN_IMAGE = "image_obstacle.jpeg"
obstacle_robot_finder = ObstacleRobotFinder()
obstacle_robot_finder.detect_obstacle_position(image=AN_IMAGE)
