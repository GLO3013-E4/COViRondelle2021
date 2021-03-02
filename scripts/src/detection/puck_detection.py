import cv2
import numpy as np
from sklearn.cluster import KMeans
from collections import Counter

from object_detection import ObjectDetection

from color_boundaries import ColorBoundaries


class PuckDetection( ObjectDetection ):

    def __init__(self, image, color):
        super().__init__( image, color, 35, 65 )
        self.minimum_area = 1440
        self.maximum_area = 2200

    def detect_puck(self):
        cv2.namedWindow( 'Color detection', cv2.WINDOW_NORMAL )
        image_copy = self.copy_image()
        puck_position = self._find_color( image_copy )
        self._show_image( image_copy )
        return puck_position

    def _find_color(self, image_copy):
        image_hsv = cv2.cvtColor( self.image, cv2.COLOR_BGR2HSV )

        color_lower_boundary = self.lower_boundary.get_lower_boundaries( self.object_to_detect )
        color_upper_boundary = self.upper_boundary.get_upper_boundaries( self.object_to_detect )

        mask = cv2.inRange( image_hsv, color_lower_boundary, color_upper_boundary )
        return self._get_contours( mask, image_copy )

    def _get_contours(self, image_mask, image_copy):
        contours, hierarchy = cv2.findContours( image_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE )

        for contour in contours:
            area = cv2.contourArea( contour )

            if self.is_in_area( area ):
                perimeter = cv2.arcLength( contour, True )
                zone_approximation = cv2.approxPolyDP( contour, 0.05 * perimeter, True )
                object_corner = len( zone_approximation )
                x_position, y_position, width, height = cv2.boundingRect( zone_approximation )

                if self.object_is_in_range( width, height ):
                    self.draw_rectangle_on_image( image_copy, x_position, y_position, width, height,
                                                  self.get_object_name( object_corner ) )
                    try:
                        puck_position = self.generate_puck_position( x_position, y_position,
                                                                     width, height )
                    except NameError:
                        puck_position = self.generate_puck_position( 0, 0, 0, 0 )
                    return puck_position
        return 0

    def get_object_name(self, object_corner):
        if object_corner >= 4:
            object_type = str( self.object_to_detect ) + " puck"
        else:
            object_type = "None"
        return object_type

    def is_in_area(self, area):
        return self.minimum_area < area < self.maximum_area


def get_dominant_color(image, k=4, image_processing_size=None):
    if image_processing_size is not None:
        image = cv2.resize( image, image_processing_size,
                            interpolation=cv2.INTER_AREA )

    image = image.reshape( (image.shape[0] * image.shape[1], 3) )

    clt = KMeans( n_clusters=k )
    labels = clt.fit_predict( image )

    label_counts = Counter( labels )

    dominant_color = clt.cluster_centers_[label_counts.most_common( 1 )[0][0]]
    return list( dominant_color )


def find_hsv_color(hsv):
    color_boundaries = ColorBoundaries()
    colors = color_boundaries.get_boundaries_dict()
    for color, boundaries in colors.items():
        if boundaries["lower"][0] <= hsv[0] <= boundaries["upper"][0] and boundaries["lower"][1] <= hsv[1] <= \
                boundaries["upper"][1] and boundaries["lower"][2] <= hsv[2] <= boundaries["upper"][2]:
            return color


def detect_pucks(image):
    img = cv2.imread( image )
    output = img.copy()
    gray = cv2.cvtColor( img, cv2.COLOR_BGR2GRAY )
    gray = cv2.medianBlur( gray, 5 )

    circles = cv2.HoughCircles( gray, cv2.HOUGH_GRADIENT, 1.2, 10, param1=50, param2=30, minRadius=25, maxRadius=30 )
    detected_circles = np.uint16( np.around( circles ) )

    for (x, y, r) in detected_circles[0].astype( np.int32 ):
        cv2.circle( output, (x, y), r, (0, 255, 0), 2 )
        cv2.circle( output, (x, y), 2, (0, 255, 255), 2 )

        roi = img[y - r: y + r, x - r: x + r]
        width, height = roi.shape[:2]
        mask = np.zeros( (width, height, 3), roi.dtype )
        cv2.circle( mask, (int( width / 2 ), int( height / 2 )), r, (255, 255, 255), -1 )
        dominant_color = get_dominant_color( roi, k=4 )

        dominant_color_np = np.uint8( [[dominant_color]] )
        hsv_dominant_color = cv2.cvtColor( dominant_color_np, cv2.COLOR_BGR2HSV )
        hsv_color = find_hsv_color(hsv_dominant_color[0][0])


        cv2.putText( output, "{}".format(hsv_color), (x - 30, y - 30),
                     cv2.FONT_HERSHEY_COMPLEX, 0.5, (255, 255, 255), 1 )
        cv2.putText(output,"({}, {})".format(x, y), (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (255, 255, 255), 1)

    cv2.imshow( 'output', output )
    cv2.waitKey( 0 )
    cv2.destroyAllWindows()


detect_pucks( "monde2.jpg" )
