import math

from handlers.release_puck_handler import get_angle_correction, get_angle_between_two_points

#TODO: test
#def get_angle_correction(robot_angle, vector_angle):
#def get_angle_between_two_points(x1, y1, x2, y2):


def test_given_1_1_when_get_angle_correction_the_angle_correction_is_good():
    robot_angle = math.pi/4
    vector_angle = math.pi/3

    angle_correction = get_angle_correction(robot_angle, vector_angle)

    assert str(angle_correction)[:4] == "0.26"
def test_given_1_2_when_get_angle_correction_the_angle_correction_is_good():
    robot_angle = math.pi/4
    vector_angle = 3*math.pi/4

    angle_correction = get_angle_correction(robot_angle, vector_angle)

    assert str(angle_correction)[:4] == "1.57"
def test_given_1_3_when_get_angle_correction_the_angle_correction_is_good():
    robot_angle = math.pi / 4

    vector_angle = 5 * math.pi / 4

    angle_correction = get_angle_correction(robot_angle, vector_angle)

    assert str(angle_correction)[:4] == "3.14"


def test_given_1_4_when_get_angle_correction_the_angle_correction_is_good():
    robot_angle = math.pi / 4

    vector_angle = 7 * math.pi / 4

    angle_correction = get_angle_correction(robot_angle, vector_angle)

    assert str(angle_correction)[:5] == "-0.78"

"""
def test_given_2_1_when_get_angle_correction_the_angle_correction_is_good():
    robot_angle = BLEH
    vector_angle = BLEH

    angle_correction = get_angle_correction(robot_angle, vector_angle)

    assert angle_correction == BLEH
    pass
def test_given_2_2_when_get_angle_correction_the_angle_correction_is_good():
    robot_angle = BLEH
    vector_angle = BLEH

    angle_correction = get_angle_correction(robot_angle, vector_angle)

    assert angle_correction == BLEH
    pass
def test_given_2_3_when_get_angle_correction_the_angle_correction_is_good():
    robot_angle = BLEH
    vector_angle = BLEH

    angle_correction = get_angle_correction(robot_angle, vector_angle)

    assert angle_correction == BLEH
    pass
def test_given_2_4_when_get_angle_correction_the_angle_correction_is_good():
    robot_angle = BLEH
    vector_angle = BLEH

    angle_correction = get_angle_correction(robot_angle, vector_angle)

    assert angle_correction == BLEH
    pass
def test_given_3_1_when_get_angle_correction_the_angle_correction_is_good():
    robot_angle = BLEH
    vector_angle = BLEH

    angle_correction = get_angle_correction(robot_angle, vector_angle)

    assert angle_correction == BLEH
    pass
def test_given_3_2_when_get_angle_correction_the_angle_correction_is_good():
    robot_angle = BLEH
    vector_angle = BLEH

    angle_correction = get_angle_correction(robot_angle, vector_angle)

    assert angle_correction == BLEH
    pass
def test_given_3_3_when_get_angle_correction_the_angle_correction_is_good():
    robot_angle = BLEH
    vector_angle = BLEH

    angle_correction = get_angle_correction(robot_angle, vector_angle)

    assert angle_correction == BLEH
    pass
def test_given_3_4_when_get_angle_correction_the_angle_correction_is_good():
    robot_angle = BLEH
    vector_angle = BLEH

    angle_correction = get_angle_correction(robot_angle, vector_angle)

    assert angle_correction == BLEH
    pass
def test_given_4_1_when_get_angle_correction_the_angle_correction_is_good():
    robot_angle = BLEH
    vector_angle = BLEH

    angle_correction = get_angle_correction(robot_angle, vector_angle)

    assert angle_correction == BLEH
    pass
def test_given_4_2_when_get_angle_correction_the_angle_correction_is_good():
    robot_angle = BLEH
    vector_angle = BLEH

    angle_correction = get_angle_correction(robot_angle, vector_angle)

    assert angle_correction == BLEH
    pass
def test_given_4_3_when_get_angle_correction_the_angle_correction_is_good():
    robot_angle = BLEH
    vector_angle = BLEH

    angle_correction = get_angle_correction(robot_angle, vector_angle)

    assert angle_correction == BLEH
    pass
def test_given_4_4_when_get_angle_correction_the_angle_correction_is_good():
    robot_angle = BLEH
    vector_angle = BLEH

    angle_correction = get_angle_correction(robot_angle, vector_angle)

    assert angle_correction == BLEH
    pass
"""