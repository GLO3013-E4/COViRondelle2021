import argparse
from scripts.capture.capture_image_from_embed_camera import capture_image_from_embed_camera
from scripts.capture.capture_image_from_path import capture_image_from_path
from scripts.mapping.map_letters import map_letters
from scripts.processing.process_image_to_grayscale import process_image_to_grayscale

ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", type=str, help="path to input image")
ap.add_argument("-e", "--embed-camera", type=bool,
                help="use embed camera to get image", default=False)
ap.add_argument("-p", "--path", type=str, help="path to tesseract.exe")
args = vars(ap.parse_args())

image = capture_image_from_embed_camera() \
    if args["embed-camera"] \
    else capture_image_from_path(args["image"])

grayscale = process_image_to_grayscale(image)

letters = map_letters(grayscale)
