import argparse

from scripts.src.detection import puck_detection
from scripts.src.mapping.color import Color

ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", help="path to the image")
ap.add_argument("-c", "--color", help="color to detect")
args = vars(ap.parse_args())

image_to_detect = Color[args["image"]]
color_to_detect = Color[args["color"]]

image_detection = puck_detection.PuckDetection(image_to_detect, color_to_detect)
coords = image_detection.detect_puck()
print(coords)
