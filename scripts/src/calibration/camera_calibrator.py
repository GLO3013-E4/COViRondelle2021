import glob
import cv2
import numpy as np
from CameraCalibration import CameraCalibration

# from camera_calibration_repository import CameraCalibrationRepository

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


class CameraCalibrator:

    def __init__(self, n_chessboard_rows, n_chessboard_columns, chessboard_square_length,
                 termination_criteria=criteria):
        self.__n_chessboard_rows = n_chessboard_rows
        self.__n_chessboard_columns = n_chessboard_columns
        self.__chessboard_square_length = chessboard_square_length
        self.__termination_criteria = termination_criteria
        self.__known_board_positions = self.__create_known_board_positions()

    def calibrate_camera(self, calibration_images_path):
        image_space_chessboards_corners = \
            self.get_image_space_chessboards_corners(calibration_images_path)
        all_known_board_positions = \
            [self.__known_board_positions for i in range(len(image_space_chessboards_corners))]
        image_size = cv2.imread(glob.glob(calibration_images_path)[0]).shape[1::-1]
        # (corners_founded), camera_matrix, distortion_coefficients, (rotation_vectors, translation_vectors)
        camera_matrix, distortion_coefficients, rotation_vectors, translation_vectors = cv2.calibrateCamera(
            all_known_board_positions, image_space_chessboards_corners, image_size, None, None
        )[1:]

        calibration_error = self.__calculate_error(
            all_known_board_positions,
            image_space_chessboards_corners,
            rotation_vectors,
            translation_vectors,
            camera_matrix,
            distortion_coefficients
        )

        return CameraCalibration(
            aspect_ratio=float(image_size[0]) / float(image_size[1]),
            camera_matrix=camera_matrix,
            distortion_coefficients=distortion_coefficients,
            error=calibration_error,
            image_width=image_size[0],
            image_height=image_size[1]
        )

    def __create_known_board_positions(self):
        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(ROWS-1,COLUMNS-1,0)
        board_positions = np.array([
            [i, j, 0] for j in range(self.__n_chessboard_columns)
            for i in range(self.__n_chessboard_rows)
        ], dtype=np.float32) * self.__chessboard_square_length
        return board_positions

    def __calculate_error(self, object_points, image_points, rotation_vectors,
                          translation_vectors, camera_matrix,
                          distortion_coefficients):
        total_error = 0

        for i in range(len(object_points)):
            image_points_2, _ = cv2.projectPoints(
                object_points[i],
                rotation_vectors[i],
                translation_vectors[i],
                camera_matrix,
                distortion_coefficients
            )
            error = cv2.norm(image_points[i], image_points_2, cv2.NORM_L2) / len(image_points_2)
            total_error += error

        return total_error / len(object_points)

    def get_image_space_chessboards_corners(self, calibration_images_path, show_images=True):
        image_space_chessboards_corners = []
        filenames = glob.glob(calibration_images_path)

        for filename in filenames:
            image = cv2.imread(filename)
            grayscaled_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            corners_founded, corners = \
                cv2.findChessboardCorners(grayscaled_image, (self.__n_chessboard_rows,
                                                             self.__n_chessboard_columns), None)

            if corners_founded:
                corners_with_better_accuracy = \
                    cv2.cornerSubPix(grayscaled_image, corners, (11, 11), (-1, -1),
                                     self.__termination_criteria)
                image_space_chessboards_corners.append(corners_with_better_accuracy)
                if show_images:
                    self.draw_chessboard_corners(image, corners_with_better_accuracy)

        return image_space_chessboards_corners

    def draw_chessboard_corners(self, chessboard_image, chessboard_corners):
        image = cv2.drawChessboardCorners(chessboard_image,
                                          (self.__n_chessboard_rows, self.__n_chessboard_columns),
                                          chessboard_corners, True)
        cv2.imshow('Chessboard corners', image)
        cv2.waitKey(50000)

  # Remove this to create a new table.json
# if __name__ == '__main__':
#     calibration = CameraCalibrator(7, 6, 3)
#     # calibration.calibrate_camera('../../data/calibrations/*.jpg')
#     save = CameraCalibrationRepository()
#     save.save_calibration(calibration.calibrate_camera('../../data/calibrations/*.jpg'), 2)


# This code calibrate the camera and return a new image (idk if we keeping this)
# if __name__ == '__main__':
#     # termination criteria
#     criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
#     # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
#     objp = np.zeros((6*7,3), np.float32)
#     objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)
#     # Arrays to store object points and image points from all the images.
#     objpoints = [] # 3d point in real world space
#     imgpoints = [] # 2d points in image plane.
#     images = glob.glob('calibration/*.jpg')
#     for fname in images:
#         img = cv.imread(fname)
#         gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
#         # Find the chess board corners
#         ret, corners = cv.findChessboardCorners(gray, (7,6), None)
#         # If found, add object points, image points (after refining them)
#         if ret == True:
#             objpoints.append(objp)
#             corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
#             imgpoints.append(corners)
#             # Draw and display the corners
#             cv.drawChessboardCorners(img, (7,6), corners2, ret)
#             cv.imshow('img', img)
#             cv.waitKey(9000)
#     cv.destroyAllWindows()
#     ret, mtx, dist, rvecs, tvecs = \
#         cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
#     img = cv.imread('calibration/calibration.jpg')
#     h, w = img.shape[:2]
#     newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
#     # undistort
#     dst = cv.undistort(img, mtx, dist, None, newcameramtx)
#     # crop the image
#     x, y, w, h = roi
#     dst = dst[y:y + h, x:x + w]
#     cv.imwrite('calibration/calibresult.png', dst)
