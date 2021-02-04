import cv2
import numpy as np

image_black = np.zeros((512, 512, 3), np.uint8)

cv2.line(image_black, (0, 0), (100, 300), (0, 255, 0), 3)
cv2.rectangle(image_black, (0, 0), (100, 200), (0, 0, 255), 3)
cv2.putText(image_black, "Hello world !", (300, 400), cv2.FONT_HERSHEY_SIMPLEX, 1, (200, 150, 200), 2)


# color detection
def empty(value):
    pass


cv2.namedWindow("TrackBars")
cv2.resizeWindow("TrackBars", 640, 240)
cv2.createTrackbar("Hue Min", "TrackBars", 0, 179, empty)
cv2.createTrackbar("Hue Max", "TrackBars", 179, 179, empty)
cv2.createTrackbar("Sat Min", "TrackBars", 0, 255, empty)
cv2.createTrackbar("Sat Max", "TrackBars", 255, 255, empty)
cv2.createTrackbar("Val Min", "TrackBars", 0, 255, empty)
cv2.createTrackbar("Val Max", "TrackBars", 255, 255, empty)

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

    cv2.imshow("Image hsv", hsv_image)
    cv2.imshow("Mask", mask)

    cv2.waitKey(1000)
