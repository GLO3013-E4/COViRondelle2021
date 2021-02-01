import argparse
import pytesseract
import cv2

ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", type=str, help="path to input image")
ap.add_argument("-p", "--path", type=str, help="path to tesseract.exe")
args = vars(ap.parse_args())
img = cv2.imread(args["image"])

pytesseract.pytesseract.tesseract_cmd = args["path"]

CUSTOM_CONFIG = r'-l eng --oem 1 --psm 11 -c tessedit_char_whitelist="ABCD" '
found_letters = pytesseract.image_to_string(img, config=CUSTOM_CONFIG)

bad_chars = [' ', '\x0c', '\n']
for i in bad_chars:
    found_letters = found_letters.replace(i, '')

letters = list(found_letters)
print(letters)

number = input('Number between 1 and 9 : ')
if 1 <= int(number) <= 9:
    print(letters[int(number) - 1])
else:
    print("Bad value")
