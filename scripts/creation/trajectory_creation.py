import argparse

from PIL import Image, ImageDraw

ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", help="path to the image")
args = vars(ap.parse_args())

image = Image.open(args["image"])
draw = ImageDraw.Draw(image)

# Params
width, height = image.size

obstacles = [
    (175, 27),
    (175, 75),
    (175, 123),
    (175, 170),
    (175, 222),
    (225, 228)
]

start = (21, 24)
end = (27, 382)

pucks = [
    (175, 27),
    (175, 75),
    (175, 123),
    (175, 170),
    (175, 222),
    (225, 228)
]

img.show()

if __name__ == '__main__':
    pass
