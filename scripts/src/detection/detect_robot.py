import cv2
import numpy as np
from pyzbar.pyzbar import decode


image = cv2.imread("qr_code.png")

for qr_code in decode(image):
    qr_codes_position = np.array([qr_code.polygon], np.int32)
    qr_codes_position = qr_codes_position.reshape((-1, 1, 2))
    cv2.polylines(image, [qr_codes_position], True, (255, 0, 255), 5)

cv2.imshow("Qr Code", np.hstack([image]))
cv2.waitKey(10000)