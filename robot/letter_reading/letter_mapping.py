import RPi.GPIO as GPIO

from capture_image_from_embed_camera import capture_image_from_embed_camera
from map_letters import map_letters
from process_image_to_grayscale import process_image_to_grayscale


def letter_mapping():
    GPIO.setmode(GPIO.BOARD)

    GPIO.setup(12, GPIO.OUT)
    servo1 = GPIO.PWM(12, 50)
    GPIO.setup(13, GPIO.OUT)
    servo2 = GPIO.PWM(13, 50)

    servo1.start(0)
    servo2.start(0)
    x_position = 7
    y_position = 6
    servo2.ChangeDutyCycle(x_position)
    servo1.ChangeDutyCycle(y_position)

    image = capture_image_from_embed_camera()

    grayscale = process_image_to_grayscale(image)

    letters = map_letters(grayscale)

    return letters
