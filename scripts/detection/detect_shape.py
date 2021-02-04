import cv2
import stack_images


class DetectShapes:
    def __init__(self, image):
        self.image_shape = cv2.imread(image)

    def detect_shapes(self):
        image_copy = self.image_shape.copy()
        image_gray = cv2.cvtColor(self.image_shape, cv2.COLOR_BGR2GRAY)
        image_blur = cv2.GaussianBlur(image_gray, (7, 7), 1)

        image_canny = cv2.Canny(image_blur, 50, 50)

        # draw contours
        self.get_contours(image_canny, image_copy)

        image_stack_canny = stack_images.stackImages(0.6,
                                                     ([self.image_shape, image_gray, image_blur],
                                                      [image_canny, image_copy,
                                                       image_copy]))

        cv2.imshow("Images stack", image_stack_canny)

        cv2.waitKey(10000)

    def get_contours(self, image_canny, image_copy):
        contours, hierarchy = cv2.findContours(image_canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        for contour in contours:
            contour_area = cv2.contourArea(contour)

            if contour_area > 20:

                perimeter = cv2.arcLength(contour, False)

                approx_corner = cv2.approxPolyDP(contour, 0.03 * perimeter, False)

                object_corner = len(approx_corner)

                x, y, width, height = cv2.boundingRect(approx_corner)

                print(width, height)
                self.detect_puck(height, image_copy, width, x, y, contour)

    def detect_puck(self, height, image_copy, width, x, y, contours):
        if 40 < width < 60 and 40 < height < 60:
            cv2.rectangle(image_copy, (x, y), (x + width, y + height), (0, 255, 0), 2)
            object_type = "Puck"
            cv2.putText(image_copy, object_type,
                        (x + (width//2), y + (height//2)), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 0, 15), 2)



detect_shapes = DetectShapes("images/camera_monde2.jpg")

detect_shapes.detect_shapes()
