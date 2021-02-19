import argparse
import cv2
import numpy as np
import math


def is_on_start_area_dimension(x, y):
    if (163 < x < 195) or (595 < x < 630):
        if (204 < y < 230) or (635 < y < 665):
            return True
    return False


def is_start_area_color(red, green, blue):
    target_colors = {"Red": (255, 0, 0), "Yellow": (255, 255, 0), "Green": (0, 200, 0)}

    def color_difference(color1, color2):
        return sum([abs(component1 - component2) for component1, component2 in zip(color1, color2)])

    my_color = (red, green, blue)
    differences = [[color_difference(my_color, target_value), target_name] for target_name, target_value in
                   target_colors.items()]
    differences.sort()  # sorted by the first element of inner lists
    my_color_name = differences[0][1]
    return my_color_name == "Green"


def detect_square(corners_list):
    res = []
    for (x, y) in corners_list:
        if not is_on_start_area_dimension(x, y):
            continue
        for (z, w) in corners_list:
            distance = math.sqrt((x - z) ** 2 + (y - w) ** 2)
            if 27 < distance < 31:
                res.append((x, y))
                res.append((z, w))
    return res


def suppress_redundant_coin(my_list):
    return list(set(my_list))


def detect_center(my_list):
    x = [p[0] for p in my_list]
    y = [p[1] for p in my_list]
    centroid = (sum(x) / len(my_list), sum(y) / len(my_list))
    return centroid


def detect_the_green_area():
    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--image", help="path to the image")
    args = vars(ap.parse_args())
    img = cv2.imread(args["image"])
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    gray = np.float32(gray)
    dst = cv2.cornerHarris(gray, 2, 3, 0.02)
    ret, dst = cv2.threshold(dst, 0.02 * dst.max(), 255, 0)
    dst = np.uint8(dst)
    ret, labels, stats, centroids = cv2.connectedComponentsWithStats(dst)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.02)
    corners = cv2.cornerSubPix(gray, np.float32(centroids), (5, 5), (-1, -1), criteria)
    start_area_corners_list = []
    for i in corners:
        x, y = i.ravel()
        b = img[int(y), int(x), 0]
        g = img[int(y), int(x), 1]
        r = img[int(y), int(x), 2]
        if not is_start_area_color(r, g, b):
            continue
        start_area_corners_list.append((x, y))

    cv2.dilate(dst, None)
    start_area_corners_list = detect_square(start_area_corners_list)
    start_area_corners_list = suppress_redundant_coin(start_area_corners_list)
    # Remove the comments bellow to see the corner on the interface
    # font = cv2.FONT_HERSHEY_SIMPLEX
    # center = detect_center(start_area_corners_list)
    # cv2.circle(img, (int(center[0]), int(center[1])), 3, (255, 0, 255), 2)
    # cv2.putText(img, str(center[0]) + ',  ' +
    #             str(center[1]), (int(center[0]), int(center[1])), font, 1, (255, 0, 255), 2)
    # for (x, y) in start_area_corners_list:
    #     cv2.circle(img, (int(x), int(y)), 3, (0, 0, 255), 2)
    #     cv2.putText(img, str(x) + ',  ' +
    #                 str(y), (int(x), int(y)), font, 1, (0, 0, 255), 2)
    # cv2.imshow('Start Area', img)
    # if cv2.waitKey(0) & 0xff == 27:
    #     cv2.destroyAllWindows()
    return start_area_corners_list
