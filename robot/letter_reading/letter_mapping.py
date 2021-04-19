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
        self.servo_y = GPIO.PWM(11, 50)
        GPIO.setup(12, GPIO.OUT)
        self.servo_x = GPIO.PWM(12, 50)

        self.servo_y.start(0)
        self.servo_y.ChangeDutyCycle(7.5)
        time.sleep(1)
        self.servo_y.stop()

    def stop_servos(self):
        pass
        # self.servo2.stop()
        # GPIO.cleanup()

    def letter_mapping(self):
        servo_x_angle = [7, 6, 5, 8, 9]
        for angle in servo_x_angle:
            letters = self.camera_panning(angle)
            if len(letters) == 9:
                return letters
            else:
                continue

    def fonction_a_oli(self):
        letters = self.camera_panning(7)
        if len(letters) == 9:
            self.stop_servos()
            return letters
        letters = self.camera_panning(5)
        if len(letters) == 9:
            self.stop_servos()
            return letters
        letters = self.camera_panning(9)
        if len(letters) == 9:
            self.stop_servos()
            return letters
        return letters

    def camera_panning(self, x_position):

        self.servo_x.start(0)
        self.servo_x.ChangeDutyCycle(x_position)
        time.sleep(1)
        self.servo_x.stop()
        time.sleep(2)

        image = capture_image_from_embed_camera()
        grayscale = process_image_to_grayscale(image)
        letters = map_letters(grayscale)
        return letters
