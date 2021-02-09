from main.src.handlers.read_image_handler import ReadImageHandler

image_path = 'imagePath'
expected_handled = 'expectedHandled'


class MockImageReader:
    @staticmethod
    def read_image_from_path(path):
        if path is image_path:
            return expected_handled


def test_when_handling_then_read_image_from_path():
    handler = ReadImageHandler(MockImageReader(), image_path)

    handled = handler.handle()

    assert handled is expected_handled
