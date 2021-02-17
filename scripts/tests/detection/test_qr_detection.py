from scripts.src.detection.qr_code_detection import QrDetection

INVALID_QR_CODE = "HelloWorld"
AN_IMAGE = "monde.jpg"
INVALID_IMAGE = "HelloWorld.jpg"

qr_detection = QrDetection(AN_IMAGE)

def test_given_invalid_qr_code_then_return_Zero():
    expected_result = qr_detection.detect_qr_code(INVALID_QR_CODE)

    assert expected_result == 0