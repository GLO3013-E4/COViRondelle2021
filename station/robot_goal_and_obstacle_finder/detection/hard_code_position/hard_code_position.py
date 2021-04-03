import cv2


def position_of_resistance_panel():
    position_x_of_resistance_panel = 150
    position_y_of_resistance_panel = 830
    return position_x_of_resistance_panel, position_y_of_resistance_panel


def position_of_control_panel():
    position_x_of_control_panel = 50
    position_y_of_control_panel = 430
    return position_x_of_control_panel, position_y_of_control_panel


def test_hard_code_position(image):
    image = cv2.imread(image)
    print(image.shape)
    position_x = 50
    position_y = 430
    cv2.putText(image, "ici", (position_x, position_y),
                cv2.FONT_HERSHEY_COMPLEX, 0.5, (255, 255, 255), 1)
    cv2.imshow('square detection', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
