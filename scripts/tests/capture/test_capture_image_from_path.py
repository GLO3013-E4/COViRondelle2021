import pytest

from scripts.capture.capture_image_from_path import capture_image_from_path


def test_given_valid_path_then_image_is_valid():
    valid_paths = [
        'data/images/command_panel_example_1.png',
        'data/images/command_panel_example_2.png'
    ]

    for valid_path in valid_paths:
        assert_image_is_valid(valid_path)


def assert_image_is_valid(path):
    image = capture_image_from_path(path)

    assert image is not None


def test_given_invalid_path_then_raise_exception():
    invalid_path = 'data/images/invalid.png'

    with pytest.raises(Exception):
        capture_image_from_path(invalid_path)
