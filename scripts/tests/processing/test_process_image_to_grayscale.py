from scripts.capture.capture_image_from_path import capture_image_from_path
from scripts.processing.process_image_to_grayscale import process_image_to_grayscale

valid_paths = [
    'data/images/command_panel_example_1.png',
    'data/images/command_panel_example_2.png'
]

images = []

for valid_path in valid_paths:
    images.append(capture_image_from_path(valid_path))


def test_given_image_then_grayscale_is_valid():
    for image in images:
        assert_grayscale_is_valid(image)


def assert_grayscale_is_valid(image):
    image = process_image_to_grayscale(image)

    assert image is not None
