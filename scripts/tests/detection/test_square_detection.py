from scripts.src.detection.square_detection import SquareDetection

A_VALID_IMAGE = "camera_monde_qr.jpg"
A_IMAGE_WITHOUT_FOUR_CORNER = "image_without_square.jpg"
A_NUMBER_LOWER_THAN_AREA = 2600
A_NUMBER_IN_AREA = 27000
A_NUMBER_HIGHER_THAN_AREA = 32000

square_detection = SquareDetection(A_VALID_IMAGE)
invalid_square_detection = SquareDetection(A_IMAGE_WITHOUT_FOUR_CORNER)


def test_given_an_area_with_number_lower_than_range_then_should_return_false():
    is_in_area = square_detection.is_in_area(A_NUMBER_LOWER_THAN_AREA)

    assert is_in_area is False


def test_given_an_area_with_number_in_range_then_should_return_false():
    is_in_area = square_detection.is_in_area(A_NUMBER_IN_AREA)

    assert is_in_area is True


def test_given_an_area_with_number_higher_than_range_then_should_return_false():
    is_in_area = square_detection.is_in_area(A_NUMBER_HIGHER_THAN_AREA)

    assert is_in_area is False


def test_given_invalid_image_position_should_return_zero():
    expected_result = invalid_square_detection.detect_square()
    assert expected_result == 0


def test_given_valid_square_object_corner_of_eight_should_return_object_name_of_square():
    expected_result = square_detection.get_object_name(8)
    assert expected_result == "square"


def test_given_invalid_square_object_corner_of_eight_should_return_object_name_of_square():
    expected_result = square_detection.get_object_name(4)
    assert expected_result == "None"