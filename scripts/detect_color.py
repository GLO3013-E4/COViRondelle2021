import numpy as np
import argparse
import cv2
import time

cv2.namedWindow('image', cv2.WINDOW_NORMAL)

ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", help = "path to the image")
args = vars(ap.parse_args())

image = cv2.imread(args["image"])

hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

boundaries = [
    ([42, 145, 55], [78, 255, 110]),
    ([0,0,0], [179,255,255])
]

for (lower, upper) in boundaries:
    lower = np.array(lower, dtype = "uint8")
    upper = np.array(upper, dtype = "uint8")
    mask = cv2.inRange(hsv, lower, upper)
    output = cv2.bitwise_and(image, image, mask = mask)

    output = cv2.resize(output, (500, 1000))

    cv2.imshow("images", np.hstack([output]))
    cv2.waitKey(0)

cv2.destroyAllWindows()