import RPi.GPIO as GPIO
import time

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


    def letter_mapping(self):
        letters = []
        letters = self.camera_panning(7.3)
        if len(letters) == 9:
            GPIO.cleanup()
            return letters
        letters = self.camera_panning(4)
        if len(letters) == 9:
            GPIO.cleanup()
            return letters
        letters = self.camera_panning(10)
        if len(letters) == 9:
            GPIO.cleanup()
            return letters

        #peut etre pas une bonne idée une exception
        if letters == []:
            raise Exception

        #retirer le code en bas?, on le fait deja dans camera_panning
        image = capture_image_from_embed_camera()

        grayscale = process_image_to_grayscale(image)

        letters = map_letters(grayscale)

        return letters


    def camera_panning(self, x_position):
        self.servo1.start(0)
        self.servo2.start(0)
        self.servo2.ChangeDutyCycle(x_position)
        self.servo1.ChangeDutyCycle(7.5)
        
        #on peut mettre un sleep pour donner du temps aux servos :
        #time.sleep(1)
        self.servo2.stop()
        self.servo1.stop()

        image = capture_image_from_embed_camera()

        grayscale = process_image_to_grayscale(image)

        letters = map_letters(grayscale)

        return letters

