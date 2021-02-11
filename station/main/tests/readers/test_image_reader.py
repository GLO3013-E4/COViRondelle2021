import pytest

from station.main.tests.util.command_panel_helper import command_panel_images_path
from station.main.src.readers.image_reader import ImageReader

image_reader = ImageReader()


def test_given_valid_path_when_reading_image_from_path_then_image_is_valid():
    valid_paths = command_panel_images_path

    for valid_path in valid_paths:
        assert_image_is_valid(valid_path)


def assert_image_is_valid(path):
    image = image_reader.read_image_from_path(path)

    assert image is not None


def test_given_invalid_path_when_reading_image_from_path_then_raise_exception():
    invalid_path = 'data/images/invalid.png'

    with pytest.raises(Exception):
        image_reader.read_image_from_path(invalid_path)
