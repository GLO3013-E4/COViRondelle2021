import RPi.GPIO as GPIO
import time

from capture_image_from_embed_camera import capture_image_from_embed_camera
from map_letters import map_letters
from process_image_to_grayscale import process_image_to_grayscale


class Mapping:
    def start_servos(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)

        GPIO.setup(11, GPIO.OUT)
        self.servo_y = GPIO.PWM(11, 50)
        GPIO.setup(12, GPIO.OUT)
        self.servo_x = GPIO.PWM(12, 50)

        self.servo_y.start(7.5)
        time.sleep(0.5)
        self.servo_y.stop()

    def stop_servos(self):
        pass
        #self.servo2.stop()
        #GPIO.cleanup()

    def letter_mapping(self):
        self.start_servos()
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
        self.servo_x.start(x_position)
        time.sleep(1)
        self.servo_x.stop()
        time.sleep(2)

        image = capture_image_from_embed_camera()
        grayscale = process_image_to_grayscale(image)
        letters = map_letters(grayscale)
        return letters
