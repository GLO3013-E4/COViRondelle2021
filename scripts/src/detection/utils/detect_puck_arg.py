import argparse

from scripts.src.detection.puck_detection import PuckDetection

ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", help="path to the image")
ap.add_argument("-c", "--color", help="color to detect")
args = vars(ap.parse_args())

image_to_detect = args["image"]
color_to_detect = args["color"]

image_detection = PuckDetection(image_to_detect, color_to_detect)
coords = image_detection.detect_puck()
print(coords)

