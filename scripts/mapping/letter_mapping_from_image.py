import argparse
import pytesseract
import cv2

ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", type=str, help="path to input image")
ap.add_argument("-p", "--path", type=str, help="path to tesseract.exe")
args = vars(ap.parse_args())
image = cv2.imread(args["image"])

pytesseract.pytesseract.tesseract_cmd = args["path"]

ret,threshold = cv2.threshold(image,127,255,cv2.THRESH_BINARY)

CUSTOM_CONFIG = r'--oem 3 --psm 11 -c tessedit_char_whitelist="ABCD"'
found_letters = pytesseract.image_to_string(threshold, config=CUSTOM_CONFIG)

print(found_letters)
good_chars = ['A', 'B', 'C', 'D']
for i in found_letters:
    if i not in good_chars:
        found_letters = found_letters.replace(i, '')

        letters = list(found_letters)
        print(letters)
    else:
        print("Bad value")

cv2.destroyAllWindows()