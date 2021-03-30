import RPi.GPIO as GPIO

from capture_image_from_embed_camera import capture_image_from_embed_camera
from process_image_to_grayscale import process_image_to_grayscale
from map_letters import map_letters


GPIO.setmode(GPIO.BOARD)

GPIO.setup(11, GPIO.OUT)
servo1 = GPIO.PWM(11, 50)
GPIO.setup(12, GPIO.OUT)
servo2 = GPIO.PWM(12, 50)

servo1.start(0)
servo2.start(0)

def letter_mapping():
    letters = []
    letters = camera_panning(7.3)
    if letters.len() == 9:
        return letters
    letters = camera_panning(4)
    if letters.len() == 9:
        return letters
    letters = camera_panning(10)
    if letters.len() == 9:
        return letters
    return []

def camera_panning(x_position):
    servo2.ChangeDutyCycle(x_position)
    servo1.ChangeDutyCycle(7.5)

    image = capture_image_from_embed_camera()

    grayscale = process_image_to_grayscale(image)

    letters = map_letters(grayscale)

    return letters
