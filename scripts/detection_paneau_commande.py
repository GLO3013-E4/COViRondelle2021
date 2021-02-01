import argparse
import pytesseract
import cv2

ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", type=str,help="path to input image")
ap.add_argument("-p", "--path", type=str,help="path to tesseract.exe")
args = vars(ap.parse_args())
img = cv2.imread(args["image"])

pytesseract.pytesseract.tesseract_cmd = args["path"]

custom_config = r'-l eng --oem 1 --psm 11 -c tessedit_char_whitelist="ABCD" '
found_letters = pytesseract.image_to_string(img, config=custom_config)

bad_chars = [' ', '\x0c', '\n']
for i in bad_chars :
    found_letters = found_letters.replace(i, '')

letters = list(found_letters)
print(letters) # TODO: Send values to station instead of printing

number = input('Numéro entre 1 et 9 : ')
if(int(number) >= 1 and int(number) <= 9):
    print (letters[int(number) -1])
else:
    print("Mauvaise valeur")
