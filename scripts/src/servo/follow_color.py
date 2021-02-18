import RPi.GPIO as GPIO
import cv2

from scripts.src.detection.lower_boundary import LowerBoundary
from scripts.src.detection.upper_boundary import UpperBoundary
from scripts.src.mapping.color import Color

GPIO.setmode(GPIO.BOARD)

GPIO.setup(12, GPIO.OUT)
servo1 = GPIO.PWM(12, 50)
GPIO.setup(33, GPIO.OUT)
servo2 = GPIO.PWM(33, 50)

servo1.start(0)
servo2.start(0)

cap = cv2.VideoCapture(0)
CAP_WIDTH = 480
CAP_HEIGHT = 320
cap.set(3, CAP_WIDTH)
cap.set(4, CAP_HEIGHT)

_, frame = cap.read()
rows, cols, _ = frame.shape

x_medium = int(cols / 2)
x_center = int(cols / 2)
y_medium = int(rows / 2)
y_center = int(rows / 2)
x_position = 90
y_position = 90

MOVEMENT_THRESHOLD = 30


def angle_to_per(angle):
    start = 2
    end = 12
    ratio = (end - start) / 180
    angle_p = angle * ratio
    return start + angle_p


def contour_area(contour):
    return cv2.contourArea(contour)


def adjust_position(original, medium, center):
    if original > 180 or original < 0:
        return original

    position = original

    if medium < center - MOVEMENT_THRESHOLD:
        position -= 1
    elif medium > center + MOVEMENT_THRESHOLD:
        position += 1
    return angle_to_per(position)


while True:
    _, frame = cap.read()
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # TODO : Could we receive another color?
    lower_red = LowerBoundary().get_lower_boundaries(Color.RED)
    upper_red = UpperBoundary().get_upper_boundaries(Color.RED)
    red_mask = cv2.inRange(hsv_frame, lower_red, upper_red)
    _, contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=contour_area, reverse=True)

    for cnt in contours:
        (x, y, w, h) = cv2.boundingRect(cnt)

        x_medium = int((x + x + w) / 2)
        y_medium = int((y + y + h) / 2)
        break

    x_position = adjust_position(x_position, x_medium, x_center)
    servo2.ChangeDutyCycle(angle_to_per(x_position))

    y_position = adjust_position(y_position, y_medium, y_center)
    servo1.ChangeDutyCycle(angle_to_per(y_position))

    cv2.line(frame, (x_medium, 0), (x_medium, CAP_WIDTH), (0, 255, 0), 2)
    cv2.line(frame, (0, y_medium), (CAP_WIDTH, y_medium), (0, 255, 0), 2)
    cv2.imshow("Frame", frame)

    if cv2.waitKey(1) == ord('a'):
        servo1.stop()
        servo2.stop()
        GPIO.cleanup()
        break

cap.release()
cv2.destroyAllWindows()
