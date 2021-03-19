import cv2
import numpy as np
import stack_images


class TrackBarDetection:

    def on_track_bar_change(self, value):
        print(self, value)

    def start_track_bar(self):
        cv2.namedWindow("TrackBars")
        cv2.resizeWindow("TrackBars", 640, 240)
        cv2.createTrackbar("Hue Min", "TrackBars", 0, 179, self.on_track_bar_change)
        cv2.createTrackbar("Hue Max", "TrackBars", 179, 179, self.on_track_bar_change)
        cv2.createTrackbar("Sat Min", "TrackBars", 0, 255, self.on_track_bar_change)
        cv2.createTrackbar("Sat Max", "TrackBars", 255, 255, self.on_track_bar_change)
        cv2.createTrackbar("Val Min", "TrackBars", 0, 255, self.on_track_bar_change)
        cv2.createTrackbar("Val Max", "TrackBars", 255, 255, self.on_track_bar_change)

        image_camera_monde = cv2.imread("robot_obstacles4.jpg")

        while True:
            hsv_image = cv2.cvtColor(image_camera_monde, cv2.COLOR_BGR2HSV)

            h_min = cv2.getTrackbarPos("Hue Min", "TrackBars")
            h_max = cv2.getTrackbarPos("Hue Max", "TrackBars")
            s_min = cv2.getTrackbarPos("Sat Min", "TrackBars")
            s_max = cv2.getTrackbarPos("Sat Max", "TrackBars")
            v_min = cv2.getTrackbarPos("Val Min", "TrackBars")
            v_max = cv2.getTrackbarPos("Val Max", "TrackBars")

            print(h_min, s_min, v_min, h_max, s_max, v_max)

            lower = np.array([h_min, s_min, v_min])
            upper = np.array([h_max, s_max, v_max])

            mask = cv2.inRange(hsv_image, lower, upper)

            image_result = cv2.bitwise_and(image_camera_monde, image_camera_monde, mask=mask)

            image_stack = stack_images.stackImages(0.6, ([image_camera_monde, hsv_image], [mask, image_result]))

            cv2.imshow("Image hsv", image_stack)

            cv2.waitKey(2000)


trackBar = TrackBarDetection()
trackBar.start_track_bar()