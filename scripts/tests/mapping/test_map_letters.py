from scripts.mapping.map_letters import map_letters
from scripts.tests.util.command_panel_helper import command_panel_images_letters, command_panel_images

images = command_panel_images
expected_letters_for_images = command_panel_images_letters


# TODO : This requires tesseract to be installed. We should use Docker for CI
def test_given_image_then_letters_are_mapped():
    for i, image in enumerate(images):
        assert_letters_are_mapped(image, expected_letters_for_images[i])


def assert_letters_are_mapped(image, expected_letters):
    letters = map_letters(image)

    assert len(letters) == len(expected_letters)
    assert all([a == b for a, b in zip(letters, expected_letters)])
