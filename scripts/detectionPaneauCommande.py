import pytesseract
import numpy as np
import cv2
import argparse

pytesseract.pytesseract.tesseract_cmd = r'C:/Program Files/Tesseract-OCR/tesseract.exe' # TODO: Change for non absolut path in RPi 

ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", type=str,help="path to input image")
args = vars(ap.parse_args())
img = cv2.imread(args["image"])

kernel = np.ones((2, 1), np.uint8)
customConfig = r'-l eng --oem 1 --psm 11 -c tessedit_char_whitelist="ABCD" '
c = pytesseract.image_to_string(img, config=customConfig)

bad_chars = [' ', '\x0c', '\n']
for i in bad_chars :
    c = c.replace(i, '')

listeLettres = list(c)
print(listeLettres) # TODO: Send values to station instead of printing

nb = input('Numéro entre 1 et 9 : ')
if(int(nb) >= 1 and int(nb) <= 9):
    print (listeLettres[int(nb) -1])
else:
    print("Mauvaise valeur")
