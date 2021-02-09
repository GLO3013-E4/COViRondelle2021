import pytest

from detection.puck_detection import PuckDetection

detection_puck = PuckDetection()

def test_given_an_area_in_range_then_return_true():
    area = 1700

    is_in_Area = detection_puck.is_in_area(area)

    assert True == is_in_Area

