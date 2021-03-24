import cv2

vid = cv2.VideoCapture(0)

vid.set(cv2.CAP_PROP_FRAME_WIDTH, 1600)
vid.set(cv2.CAP_PROP_FRAME_HEIGHT, 904)

success, image = vid.read()
count = 0
success = True
while success:
    success, image = vid.read()
    print('Read a new frame: ', success)
    cv2.imwrite("frame%d.jpg" % count, image)  # save frame as JPEG file
    count += 1
    break

vid.release()
