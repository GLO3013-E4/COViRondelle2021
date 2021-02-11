import argparse
import cv2
import numpy as np


def is_start_area(red, green, blue):
    target_colors = {"Red": (255, 0, 0), "Yellow": (255, 255, 0), "Green": (0,200,0)}

    def color_difference(color1, color2):
        return sum([abs(component1 - component2) for component1, component2 in zip(color1, color2)])

    my_color = (red, green, blue)
    differences = [[color_difference(my_color, target_value), target_name] for target_name, target_value in
                   target_colors.items()]
    differences.sort()  # sorted by the first element of inner lists
    my_color_name = differences[0][1]
    return my_color_name == "Green"


ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", help="path to the image")
args = vars(ap.parse_args())
img = cv2.imread(args["image"])
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

gray = np.float32(gray)
dst = cv2.cornerHarris(gray, 2, 3, 0.01)
ret, dst = cv2.threshold(dst, 0.01*dst.max(), 255, 0)
dst = np.uint8(dst)
ret, labels, stats, centroids = cv2.connectedComponentsWithStats(dst)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.01)
corners = cv2.cornerSubPix(gray, np.float32(centroids), (5, 5), (-1, -1), criteria)
font = cv2.FONT_HERSHEY_SIMPLEX
for i in corners:
    x, y = i.ravel()
    b = img[int(y), int(x), 0]
    g = img[int(y), int(x), 1]
    r = img[int(y), int(x), 2]
    if not is_start_area(r, g, b):
        continue
    cv2.circle(img, (int(x), int(y)), 3, (255, 0, 255), 2)
    # print(x, y)
    # cv2.putText(img, str(r) + ',' +
    #                 str(g) + ',' + str(b), (int(x), int(y)), font, 1, (0, 0, 255), 2)


dst = cv2.dilate(dst, None)
# Threshold for an optimal value, it may vary depending on the image.
# img[dst > 0.03 * dst.max()] = [255, 255, 0]
cv2.imshow('Start Area', img)
if cv2.waitKey(0) & 0xff == 27:
    cv2.destroyAllWindows()
