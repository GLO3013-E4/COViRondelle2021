import cv2
import numpy as np
from object_detection import ObjectDetection

from lower_boundary import LowerBoundary
from upper_boundary import UpperBoundary


class PuckDetection(ObjectDetection):

    def __init__(self, image, color):
        super().__init__(image, color, 35, 65)
        self.minimum_area = 1440
        self.maximum_area = 2200

    def detect_puck(self):
        cv2.namedWindow('Color detection', cv2.WINDOW_NORMAL)
        image_copy = self.copy_image()
        puck_position = self._find_color(image_copy)
        self._show_image(image_copy)
        return puck_position

    def _find_color(self, image_copy):
        image_hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)

        color_lower_boundary = self.lower_boundary.get_lower_boundaries( self.object_to_detect )
        color_upper_boundary = self.upper_boundary.get_upper_boundaries( self.object_to_detect )

        mask = cv2.inRange(image_hsv, color_lower_boundary, color_upper_boundary)
        return self._get_contours(mask, image_copy)


    def _get_contours(self, image_mask, image_copy):
        contours, hierarchy = cv2.findContours(image_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        for contour in contours:
            area = cv2.contourArea(contour)

            if self.is_in_area(area):
                perimeter = cv2.arcLength(contour, True)
                zone_approximation = cv2.approxPolyDP(contour, 0.05 * perimeter, True)
                object_corner = len(zone_approximation)
                x_position, y_position, width, height = cv2.boundingRect(zone_approximation)

                if self.object_is_in_range(width, height):
                    self.draw_rectangle_on_image(image_copy, x_position, y_position, width, height,
                                                 self.get_object_name(object_corner))
                    try:
                        puck_position = self.generate_puck_position(x_position, y_position,
                                                                    width, height)
                    except NameError:
                        puck_position = self.generate_puck_position(0, 0, 0, 0)
                    return puck_position
        return 0

    def find_color_updated(self, image):
        red_lower = LowerBoundary()
        red_upper = UpperBoundary()
        image = cv2.imread(image)
        original = image.copy()
        image = cv2.cvtColor( image, cv2.COLOR_BGR2HSV )
        lower = np.array(red_lower.get_lower_boundaries("green"), dtype="uint8" )
        upper = np.array( red_upper.get_upper_boundaries("green"), dtype="uint8" )
        mask = cv2.inRange( image, lower, upper )

        # Find contours
        cnts = cv2.findContours( mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE )
        # Extract contours depending on OpenCV version
        cnts = cnts[0] if len( cnts ) == 2 else cnts[1]

        # Iterate through contours and filter by the number of vertices
        for c in cnts:
            perimeter = cv2.arcLength( c, True )
            approx = cv2.approxPolyDP( c, 0.04 * perimeter, True )
            if len( approx ) > 5:
                cv2.drawContours( original, [c], -1, (36, 255, 12), -1 )

        cv2.imshow( 'mask', mask )
        cv2.imshow( 'original', original )
        cv2.imwrite( 'mask.png', mask )
        cv2.imwrite( 'original.png', original )
        cv2.waitKey()




    def get_object_name(self, object_corner):
        if object_corner >= 4:
            object_type = str( self.object_to_detect ) + " puck"
        else:
            object_type = "None"
        return object_type

    def is_in_area(self, area):
        return self.minimum_area < area < self.maximum_area



COLOR_NAMES = ["red", "orange", "yellow", "green",  "blue", "purple", "red "]

COLOR_RANGES_HSV = {
    "red": [(0, 216, 92), (179, 255, 106)],
    "orange": [(10, 50, 10), (25, 255, 255)],
    "yellow": [(25, 50, 10), (35, 255, 255)],
    "white" : [(20, 0, 155), (124, 62, 186)],
    "green": [(35, 50, 10), (80, 255, 255)],
    "cyan": [(80, 50, 10), (100, 255, 255)],
    "blue": [(100, 50, 10), (130, 255, 255)],
    "purple": [(130, 50, 10), (170, 255, 255)],
    "red ": [(170, 50, 10), (180, 255, 255)]
}



def detectCirclesWithDp( frame, dp=1):
        blurred = cv2.medianBlur( frame, 25 )
        grayMask = cv2.cvtColor( blurred, cv2.COLOR_BGR2GRAY )
        # cannyMask = cv2.Canny(grayMask, 50, 240)
        return cv2.HoughCircles( grayMask, cv2.HOUGH_GRADIENT, dp, 40, param1=10, param2=30, minRadius=20,
                                 maxRadius=70 )

def getROI(frame, x, y, r):
    return frame[int( y - r / 2 ):int( y + r / 2 ), int( x - r / 2 ):int( x + r / 2 )]

def getMask(frame, color):
    blurredFrame = cv2.GaussianBlur( frame, (3, 3), 0 )
    hsvFrame = cv2.cvtColor( blurredFrame, cv2.COLOR_BGR2HSV )

    colorRange = COLOR_RANGES_HSV[color]
    lower = np.array( colorRange[0] )
    upper = np.array( colorRange[1] )

    colorMask = cv2.inRange( hsvFrame, lower, upper )
    colorMask = cv2.bitwise_and( blurredFrame, blurredFrame, mask=colorMask )

    return colorMask

def getDominantColor(roi):
    roi = np.float32( roi )

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
    K = 4
    ret, label, center = cv2.kmeans( roi, K, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS )

    center = np.uint8( center )
    res = center[label.flatten()]
    res2 = res.reshape( roi.shape )

    pixelsPerColor = []
    for color in COLOR_NAMES:
        mask = getMask( res2, color )
        greyMask = cv2.cvtColor( mask, cv2.COLOR_BGR2GRAY )
        count = cv2.countNonZero( greyMask )
        pixelsPerColor.append( count )

    return COLOR_NAMES[pixelsPerColor.index( max( pixelsPerColor ) )]


def detect(image):
    image = cv2.imread(image)
    image_copy = image.copy()
    circles = detectCirclesWithDp(image)
    if circles is not None:
        for circle in circles[0, :]:
            roi = getROI(image_copy, circle[0], circle[1], circle[2])
            color = getDominantColor(roi)
            cv2.circle(image, (circle[0], circle[1]), 20, (255, 0,0), 1)
            cv2.circle(image, (circle[0], circle[1]), 2,(255, 0,0), 2)
            cv2.putText(image, color, (int(circle[0] + 40), int(circle[1] + 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                           (255, 0,0))

    while True:
        cv2.imshow("frame", image)
        if cv2.waitKey(20) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()






#puck_detection = PuckDetection("monde2.jpg", "qwder")
#puck_detection.find_color_updated("camera_monde_qr.jpg")
detect("monde2.jpg")

