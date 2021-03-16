from obstacle_detection import ObstacleDetection
from src.calibration.camera_calibration_repository import CameraCalibrationRepository


class ObstacleRobotFinder:
    def __init__(self):
        self.obstacle_detection = ObstacleDetection()

    def detect_obstacle_position(self, image):
        aruco_marker_width = 100 # mm
        image_width = 1600
        image_height = 904
        dimension = (image_width, image_height)

        camera_matrix = [
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

        distortion_coefficients =  [
        [
            4.767177288453392,
            0.016944982578597136,
            0.009067230348369367,
            0.03617489894832692,
            8.084050600445835e-06
        ]
    ]


        obstacles_position = self.obstacle_detection.detect_obstacle(image);
        obstacle_3d_position = self.obstacle_detection\
            .calculate_3D_position(obstacles_position=obstacles_position,
                                   aruco_marker_width=aruco_marker_width,
                                   camera_matrix=camera_matrix,
                                   distortion_coefficients=distortion_coefficients)






        obstacle_position = []



obstacle_robot_finder = ObstacleRobotFinder