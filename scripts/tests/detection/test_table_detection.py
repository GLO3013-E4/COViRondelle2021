from scripts.src.detection.table_detection import TableDetection

A_VALID_IMAGE = "camera_monde_qr.jpg"
A_NUMBER_LOWER_THAN_AREA = 1100000
A_NUMBER_IN_AREA = 1190000
A_NUMBER_HIGHER_THAN_AREA = 1300000
X_POSITION = 10
Y_POSITION = 10
A_HEIGHT = 2000
A_WIDTH = 5000


table_detection = TableDetection(A_VALID_IMAGE)


def test_given_an_area_with_number_lower_than_range_then_should_return_false():
    is_in_area = table_detection.is_in_area(A_NUMBER_LOWER_THAN_AREA)

    assert is_in_area is False



def test_given_an_area_with_number_in_range_then_should_return_false():
    is_in_area = table_detection.is_in_area(A_NUMBER_IN_AREA)

    assert is_in_area is True


def test_given_an_area_with_number_higher_then_range_then_should_return_false():
    is_in_area = table_detection.is_in_area(A_NUMBER_HIGHER_THAN_AREA)

    assert is_in_area is False


def test_given_valid_table_object_corner_of_four_should_return_object_name_of_table():
    expected_result = table_detection.get_object_name(4)
    assert expected_result == "table"


def test_given_invalid_table_object_corner_of_eight_should_return_object_name_of_none():
    expected_result = table_detection.get_object_name(2)
    assert expected_result == "None"


def test_given_table_when_generate_four_corner_then_point_a_and_point_b_should_have_same_x():
    expected_result = table_detection.generate_four_corners\
        (X_POSITION, Y_POSITION, A_WIDTH, A_HEIGHT)

    assert expected_result["corner_A"].get_position_x() == \
           expected_result["corner_D"].get_position_x()


def test_given_table_when_generate_four_corner_then_point_c_and_point_d_should_have_same_x():
    expected_result = table_detection.generate_four_corners\
        (X_POSITION, Y_POSITION, A_WIDTH, A_HEIGHT)

    assert expected_result["corner_B"].get_position_x() == \
           expected_result["corner_C"].get_position_x()



def test_given_table_when_generate_four_corner_then_point_a_and_point_d_should_have_same_y():
    expected_result = table_detection.generate_four_corners\
        (X_POSITION, Y_POSITION, A_WIDTH, A_HEIGHT)

    assert expected_result["corner_A"].get_position_y() == \
           expected_result["corner_B"].get_position_y()


def test_given_table_when_generate_four_corner_then_point_b_and_point_c_should_have_same_y():
    expected_result = table_detection.generate_four_corners\
        (X_POSITION, Y_POSITION, A_WIDTH, A_HEIGHT)

    assert expected_result["corner_C"].get_position_y() == \
           expected_result["corner_D"].get_position_y()
