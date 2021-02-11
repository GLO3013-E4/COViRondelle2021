import argparse

import puck_detection

ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", help="path to the image")
ap.add_argument("-c", "--color", help="color to detect")
args = vars(ap.parse_args())

image_to_detect = args["image"]
color_to_detect = args["color"]

image_detection = puck_detection.PuckDetection(image_to_detect, color_to_detect)
coords = image_detection.detect_puck()
print(coords)
