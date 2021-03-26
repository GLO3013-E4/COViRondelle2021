import cv2


def process_image_to_grayscale(image):
    _, grayscale = cv2.threshold(image, 127, 255, cv2.THRESH_BINARY)

    return grayscale
