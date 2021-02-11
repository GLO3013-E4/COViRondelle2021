from robot.main.src.handlers.read_image_handler import ReadImageHandler

IMAGE_PATH = 'IMAGE_PATH'
EXPECTED_HANDLED_DATA = 'EXPECTED_HANDLED_DATA'


class MockImageReader:
    @staticmethod
    def read_image_from_path(path):
        if path is IMAGE_PATH:
            return EXPECTED_HANDLED_DATA

        return None


def test_when_handling_then_read_image_from_path():
    handler = ReadImageHandler(MockImageReader(), IMAGE_PATH)

    handled_data = handler.handle()

    assert handled_data is EXPECTED_HANDLED_DATA
