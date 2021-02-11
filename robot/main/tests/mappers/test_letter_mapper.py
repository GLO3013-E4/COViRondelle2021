from robot.main.tests.util.command_panel_helper import command_panel_images_letters, \
    command_panel_images
from robot.main.src.mappers.letter_mapper import LetterMapper

images = command_panel_images
expected_letters_for_images = command_panel_images_letters

letter_mapper = LetterMapper()


def test_when_mapping_letters_from_image_then_letters_are_mapped():
    for i, image in enumerate(images):
        assert_letters_are_mapped(image, expected_letters_for_images[i])


def assert_letters_are_mapped(image, expected_letters):
    letters = letter_mapper.map_letters_from_image(image)

    assert len(letters) == len(expected_letters)
    assert all([a == b for a, b in zip(letters, expected_letters)])
