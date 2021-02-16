import RPi.GPIO as GPIO
import cv2
import numpy as np
import time

GPIO.setmode(GPIO.BOARD)

GPIO.setup(12,GPIO.OUT)
servo1 = GPIO.PWM(12,50)
GPIO.setup(33,GPIO.OUT)
servo2 = GPIO.PWM(33,50)

def angle_to_per(angle):
    start = 2
    end = 12
    ratio = (end-start)/180
    angle_p = angle*ratio
    return start + angle_p

servo1.start(0)
servo2.start(0)

cap = cv2.VideoCapture(0)
cap.set(3, 480)
cap.set(4, 320)

_, frame = cap.read()
rows, cols, _ = frame.shape

x_medium = int(cols / 2)
x_center = int(cols / 2)
y_medium = int(rows / 2)
y_center = int(rows / 2)
x_position = 90
y_position = 90

while True:
    _, frame = cap.read()
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # red color prendre en parametre la couleur voulu?
    low_red = np.array([161, 155, 84])
    high_red = np.array([179, 255, 255])
    red_mask = cv2.inRange(hsv_frame, low_red, high_red)
    _, contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=lambda x:cv2.contourArea(x), reverse=True)
    
    
    for cnt in contours:
        (x, y, w, h) = cv2.boundingRect(cnt)
        
        x_medium = int((x + x + w) / 2)
        y_medium = int((y + y + h) / 2)
        break
    
    
    if x_medium < x_center - 30:
        x_position -= 1
    elif x_medium > x_center + 30:
        x_position += 1
    servo2.ChangeDutyCycle(angle_to_per(x_position))
    
    if y_medium < y_center - 30:
        y_position += 1
    elif y_medium > y_center + 30:
        y_position -= 1
    servo1.ChangeDutyCycle(angle_to_per(y_position))
        
    cv2.line(frame, (x_medium, 0), (x_medium, 480), (0, 255, 0), 2)
    cv2.line(frame, (0, y_medium), (480, y_medium), (0, 255, 0), 2)
    cv2.imshow("Frame", frame)
    
    if cv2.waitKey(1) == ord('a'):
        servo1.stop()
        servo2.stop()
        GPIO.cleanup()
        break
    
cap.release()
cv2.destroyAllWindows()


