import cv2
import numpy as np
from pyzbar.pyzbar import decode


image = cv2.imread("qr_code.png")

for qr_code in decode(image):
    position = qr_code.rect
    print(position.left)
    cv2.rectangle(image, (position.top, 100),
                  (position.top + position.width, position.left + position.height),
                  (0, 255, 0), 2)


cv2.imshow("Qr Code", np.hstack([image]))
cv2.waitKey(10000)