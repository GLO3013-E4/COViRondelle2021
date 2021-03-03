import cv2
import numpy as np
from scripts.src.detection.object_detection import ObjectDetection
from scripts.src.detection.utils.point import Point


class SquareDetection( ObjectDetection ):

    def __init__(self, image):
        super().__init__( image, "square", 440, 550 )
        self.minimum_area = 26000
        self.maximum_area = 28000

    def detect_square(self):
        img = cv2.imread(self.image)
        cv2.imshow( "canny", img )
        return self._find_color(img)

    def _find_color(self, img):
        image_hsv = cv2.cvtColor( self.image, cv2.COLOR_BGR2HSV )

        color_lower_boundary = self.lower_boundary.get_lower_boundaries( self.object_to_detect )
        color_upper_boundary = self.upper_boundary.get_upper_boundaries( self.object_to_detect )

        mask = cv2.inRange( image_hsv, color_lower_boundary, color_upper_boundary )
        return self._get_contours( mask, img )

    @staticmethod
    def generate_four_corners(x_position, y_position, width, height):
        corner_a = Point( x_position + width, y_position )
        corner_b = Point( x_position + width, y_position + height )
        corner_c = Point( x_position, y_position + height )
        corner_d = Point( x_position, y_position )

        four_corners = {
            "corner_A": corner_a,
            "corner_B": corner_b,
            "corner_C": corner_c,
            "corner_D": corner_d
        }
        return four_corners

    def _get_contours(self, image_mask, img):
        contours, hierarchy = cv2.findContours( image_mask, cv2.RETR_EXTERNAL,
                                                cv2.CHAIN_APPROX_NONE )
        corner_position = self.generate_four_corners( 0, 0, 0, 0 )

        for contour in contours:
            area = cv2.contourArea( contour )
            if self.is_in_area( area ):
                perimeter = cv2.arcLength( contour, True )
                zone_approximation = cv2.approxPolyDP( contour, 0.05 * perimeter, True )
                x_position, y_position, width, height = cv2.boundingRect( zone_approximation )
                cv2.rectangle(img, (x_position, y_position), (x_position+width, y_position+height),
                              (0, 255, 0), 2)
                if self.object_is_in_range(width, height):
                    corner_position = self.generate_four_corners(x_position, y_position,
                                                                  width, height)

        return corner_position

    def get_object_name(self, object_corner):
        if object_corner == 8:
            object_type = str( self.object_to_detect )
        else:
            object_type = "None"
        return object_type

    def is_in_area(self, area):
        return self.minimum_area < area < self.maximum_area


def get_contours(img, imgContour):
    contours, hierarchy = cv2.findContours( img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE )
    for cnt in contours:
        area = cv2.contourArea( cnt )
        if area > 200000:
            cv2.drawContours( imgContour, cnt, -1, (255, 255, 0), 2 )
            peri = cv2.arcLength( cnt, True )
            approx = cv2.approxPolyDP( cnt, 0.02 * peri, True )
            print(len(approx))
            x, y, w, h = cv2.boundingRect(approx)
            cv2.rectangle(imgContour, (x, y), (x+w, y+h), (0, 255, 0), 2)


def detect_square(image):
    img = cv2.imread( image )
    imgContour = img.copy()
    imgBlur = cv2.GaussianBlur( img, (7, 7), 1 )
    imgGray = cv2.cvtColor( imgBlur, cv2.COLOR_BGR2GRAY )

    dst = cv2.cornerHarris(imgGray, 2, 3, 0.02)
    ret, dst = cv2.threshold(dst, 0.02 * dst.max(), 255, 0)
    dst = np.uint8(dst)
    ret, labels, stats, centroids = cv2.connectedComponentsWithStats(dst)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.02)
    corners = cv2.cornerSubPix(imgGray, np.float32(centroids), (5, 5), (-1, -1), criteria)
    print(corners)

    imgCanny = cv2.Canny(imgGray, 90, 90)
    kernel = np.ones((5,5))
    imgDil = cv2.dilate( imgCanny, kernel, iterations=1 )

    get_contours( imgDil, imgContour )

    cv2.imshow( "canny", imgContour )
    cv2.waitKey( 0 )
    cv2.destroyAllWindows()


detect_square( "monde10.jpg" )

#square_detection = SquareDetection("monde10.jpg")
#square_detect = square_detection.detect_square()
