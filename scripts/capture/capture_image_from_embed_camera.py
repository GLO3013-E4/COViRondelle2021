import cv2

stop_key = 'q'  # TODO : Is there a better key we could use?


# TODO : Test this
def capture_image_from_embed_camera():
    capture = cv2.VideoCapture(0)

    if not capture.isOpened():
        print("Could not open camera")

    ret, frame = capture.read()

    while True:
        cv2.imshow('preview', frame)
        if cv2.waitKey(1) & 0xFF == ord(stop_key):
            break

    capture.release()
    cv2.destroyAllWindows()

    return frame
