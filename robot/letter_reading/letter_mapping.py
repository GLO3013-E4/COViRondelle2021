import time
import RPi.GPIO as GPIO

from capture_image_from_embed_camera import capture_image_from_embed_camera
from map_letters import map_letters
from process_image_to_grayscale import process_image_to_grayscale


class Mapping:
    def __init__(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)

        GPIO.setup(11, GPIO.OUT)
        self.servo1 = GPIO.PWM(11, 50)
        GPIO.setup(12, GPIO.OUT)
        self.servo2 = GPIO.PWM(12, 50)

        self.servo1.start(0)
        self.servo1.ChangeDutyCycle(7)
        time.sleep(0.5)
        self.servo1.stop()

    def stop_servos(self):
        self.servo1.stop()
        self.servo2.stop()
        GPIO.cleanup()

    def letter_mapping(self):
        for i in range(2,12):
            letters = self.camera_panning(i)
            if len(letters) == 9:
                self.stop_servos()
                return letters

        return letters

    def camera_panning(self, x_position):
        self.servo2.start(0)
        self.servo2.ChangeDutyCycle(x_position)
        time.sleep(0.5)
        self.servo2.stop()

        image = capture_image_from_embed_camera()
        grayscale = process_image_to_grayscale(image)
        letters = map_letters(grayscale)

        return letters
