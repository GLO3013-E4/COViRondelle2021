import os
from collections import Counter
import cv2
import numpy as np
from sklearn.cluster import KMeans
from scripts.src.detection.color_boundaries import ColorBoundaries


class PuckDetection:

    def __init__(self):
        self.color_boundaries = ColorBoundaries()

    @staticmethod
    def copy_image(image):
        try:
            img = image.copy()
        except AttributeError as invalid_image:
            raise AttributeError("L'image est invalide") from invalid_image
        return img

    def detect_pucks(self, image, color, Debug=True):
        script_dir = os.path.dirname(__file__)
        rel_path = image
        abs_file_path = os.path.join(script_dir, rel_path)
        img = cv2.imread(abs_file_path)

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = cv2.medianBlur(gray, 5)


        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1.2, 10, param1=50,
                                   param2=30, minRadius=23,maxRadius=30)
        detected_circles = np.uint16(np.around(circles))

        wanted_puck_position = {}
        other_pucks_positions = []

        for (x, y, radius) in detected_circles[0].astype(np.int32):
            roi = img[y - radius: y + radius, x - radius: x + radius]
            width, height = roi.shape[:2]
            mask = np.zeros((width, height, 3), roi.dtype)

            cv2.circle(mask, (int(width / 2), int(height / 2)), radius,
                       (255, 255, 255), -1)

            dominant_color = self.get_dominant_color(roi, k=4)

            dominant_color_np = np.uint8([[dominant_color]])
            hsv_dominant_color = cv2.cvtColor(dominant_color_np, cv2.COLOR_BGR2HSV)
            hsv_color = self.find_hsv_color(hsv_dominant_color[0][0])

            if hsv_color == color:
                wanted_puck_position["center_position"] = (x, y)
                wanted_puck_position["radius"] = radius
                break

        for (x, y, radius) in detected_circles[0].astype(np.int32):
            cv2.circle(img, (x, y), radius, (255, 0, 0), 1)
            if wanted_puck_position["center_position"] != (x, y):
                other_pucks_positions.append({"center_position" : (x, y), "radius" : radius})

        if Debug:
            print("Debug mode is on")
            self.draw_on_image(hsv_color, img, radius, x, y)
            self.show_image(img)

        return wanted_puck_position, other_pucks_positions

    def show_image(self, output):
        cv2.imshow('output', output)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def draw_on_image(self, hsv_color, output, r, x, y):
        cv2.circle( output, (x, y), r, (0, 255, 0), 2 )
        cv2.circle( output, (x, y), 2, (0, 255, 255), 2 )
        cv2.putText( output, "{}".format( hsv_color ), (x - 30, y - 30),
                     cv2.FONT_HERSHEY_COMPLEX, 0.5, (255, 255, 255), 1 )
        cv2.putText( output, "({}, {})".format( x, y ), (x, y),
                     cv2.FONT_HERSHEY_COMPLEX, 0.5, (255, 255, 255), 1 )

    def get_dominant_color(self, image, k=4, image_processing_size=None):
        if image_processing_size is not None:
            image = cv2.resize( image, image_processing_size,
                                interpolation=cv2.INTER_AREA)

        image = image.reshape( (image.shape[0] * image.shape[1], 3) )

        clt = KMeans(n_clusters=k)
        labels = clt.fit_predict(image)
        label_counts = Counter(labels)

        dominant_color = clt.cluster_centers_[label_counts.most_common(1)[0][0]]
        return list(dominant_color)

    def find_hsv_color(self, hsv):
        colors = self.color_boundaries.get_boundaries_dict()
        for color, boundaries in colors.items():
            if boundaries["lower"][0] <= hsv[0] <= boundaries["upper"][0] and \
                    boundaries["lower"][1] <= hsv[1] <= \
                    boundaries["upper"][1] and boundaries["lower"][2] <= hsv[2] \
                    <= boundaries["upper"][2]:
                return color
        return "None"
