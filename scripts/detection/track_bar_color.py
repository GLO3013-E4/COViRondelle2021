import cv2
import numpy as np
import stack_images

image_black = np.zeros((512, 512, 3), np.uint8)

# just some reminder functions

cv2.line(image_black, (0, 0), (100, 300), (0, 255, 0), 3)
cv2.rectangle(image_black, (0, 0), (100, 200), (0, 0, 255), 3)
cv2.putText(image_black, "Hello world !", (300, 400), cv2.FONT_HERSHEY_SIMPLEX, 1, (200, 150, 200), 2)


class TrackBarDetection:
    def empty(self, value):
        pass
    # color detection

    def start_track_bar(self):
        cv2.namedWindow("TrackBars")
        cv2.resizeWindow("TrackBars", 640, 240)
        cv2.createTrackbar("Hue Min", "TrackBars", 0, 179, self.empty)
        cv2.createTrackbar("Hue Max", "TrackBars", 179, 179, self.empty)
        cv2.createTrackbar("Sat Min", "TrackBars", 0, 255, self.empty)
        cv2.createTrackbar("Sat Max", "TrackBars", 255, 255, self.empty)
        cv2.createTrackbar("Val Min", "TrackBars", 0, 255, self.empty)
        cv2.createTrackbar("Val Max", "TrackBars", 255, 255, self.empty)

        image_camera_monde = cv2.imread("images/camera_monde_exemple1.jpg")

        while True:
            hsv_image = cv2.cvtColor(image_camera_monde, cv2.COLOR_BGR2HSV)

            h_min = cv2.getTrackbarPos("Hue Min", "TrackBars")
            h_max = cv2.getTrackbarPos("Hue Max", "TrackBars")
            s_min = cv2.getTrackbarPos("Sat Min", "TrackBars")
            s_max = cv2.getTrackbarPos("Sat Max", "TrackBars")
            v_min = cv2.getTrackbarPos("Val Min", "TrackBars")
            v_max = cv2.getTrackbarPos("Val Max", "TrackBars")

            print(h_min, h_max, s_min, s_max, v_min, v_max)

            lower = np.array([h_min, s_min, v_min])
            upper = np.array([h_max, s_max, v_max])

            mask = cv2.inRange(hsv_image, lower, upper)

            image_result = cv2.bitwise_and(image_camera_monde, image_camera_monde, mask=mask)

            image_stack = stack_images.stackImages(0.6, ([image_camera_monde, hsv_image], [mask, image_result]))

            cv2.imshow("Image hsv", image_stack)

            cv2.waitKey(2000)


trackBar = TrackBarDetection()

trackBar.start_track_bar()