from main.src.handlers.map_letters_handler import MapLettersHandler
from main.tests.util.command_panel_helper import command_panel_images

sent_image = command_panel_images[0]
EXPECTED_MAPPED_LETTERS = ['L', 'O', 'L']


class MockLetterMapper:
    @staticmethod
    def map_letters_from_image(image):
        if image is sent_image:
            return EXPECTED_MAPPED_LETTERS

        return None


def test_when_handling_then_map_letters_from_image():
    handler = MapLettersHandler(MockLetterMapper())

    handled_data = handler.handle(sent_image)

    assert handled_data is EXPECTED_MAPPED_LETTERS
