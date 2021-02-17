from scripts.src.detection.qr_code_detection import QrDetection

INVALID_QR_CODE = "HelloWorld"
ROBOT_TYPE = "robot"
AN_IMAGE = "monde_qr.jpg"
INVALID_IMAGE = "HelloWorld.jpg"

qr_detection = QrDetection(AN_IMAGE)

def test_given_invalid_qr_code_then_return_Zero():
    expected_result = qr_detection.detect_qr_code(INVALID_QR_CODE)

    assert expected_result == 0


def test_given_valid_image_when_detect_robot_should_return_dictionary_og_length_four():
    expected_result = qr_detection.detect_qr_code(ROBOT_TYPE)

    assert len(expected_result) == 4



