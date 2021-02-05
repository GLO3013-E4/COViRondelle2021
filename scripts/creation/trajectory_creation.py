import argparse

from PIL import Image, ImageDraw

ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", help="path to the image")
args = vars(ap.parse_args())

image = Image.open(args["image"])

if __name__ == '__main__':
    pass
