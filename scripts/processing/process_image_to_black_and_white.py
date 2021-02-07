import cv2


# TODO : Test this
def process_image_to_black_and_white(image):
    ret, threshold = cv2.threshold(image, 127, 255, cv2.THRESH_BINARY)

    return threshold
