import pytesseract
import cv2

capture = cv2.VideoCapture(0)

if not capture.isOpened():
    print("Could not open camera")


ret,frame = capture.read()
ret,threshold = cv2.threshold(frame,127,255,cv2.THRESH_BINARY)

while True:
    cv2.imshow('preview', threshold)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

CUSTOM_CONFIG = r'--oem 3 --psm 11 -c tessedit_char_whitelist="ABCD"'
found_letters = pytesseract.image_to_string(threshold, config=CUSTOM_CONFIG)

print(found_letters)
good_chars = ['A', 'B', 'C', 'D']
for i in found_letters:
    if i not in good_chars:
        found_letters = found_letters.replace(i, '')

letters = list(found_letters)
print(letters)

number = input('Number between 1 and 9 : ')
if 1 <= int(number) <= 9:
    print(letters[int(number) - 1])
else:
    print("Bad value")

capture.release()
cv2.destroyAllWindows()
