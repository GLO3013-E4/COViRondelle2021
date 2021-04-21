import math

from handlers.release_puck_handler import get_angle_correction, get_angle_between_two_points


def test_given_4_when_get_angle_between_two_points_then_angle_is_good():
    x1, y1 = (0, 0)
    x2, y2 = (1, 1)

    angle = get_angle_between_two_points(x1, y1, x2, y2)

    assert str(angle)[:5] == "-0.78"


def test_given_1_when_get_angle_between_two_points_then_angle_is_good():
    x1, y1 = (0, 0)
    x2, y2 = (1, -1)

    angle = get_angle_between_two_points(x1, y1, x2, y2)

    assert str(angle)[:4] == "0.78"


def test_given_2_when_get_angle_between_two_points_then_angle_is_good():
    x1, y1 = (0, 0)
    x2, y2 = (-1, -1)

    angle = get_angle_between_two_points(x1, y1, x2, y2)

    assert str(angle)[:4] == "2.35"


def test_given_3_when_get_angle_between_two_points_then_angle_is_good():
    x1, y1 = (0, 0)
    x2, y2 = (-1, 1)

    angle = get_angle_between_two_points(x1, y1, x2, y2)

    assert str(angle)[:5] == "-2.35"


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

    assert str(angle_correction)[:5] == "-1.57"


def test_given_2_1_when_get_angle_correction_the_angle_correction_is_good():
    robot_angle = 3*math.pi / 4

    vector_angle = 1 * math.pi / 4

    angle_correction = get_angle_correction(robot_angle, vector_angle)

    assert str(angle_correction)[:5] == "-1.57"


def test_given_2_2_when_get_angle_correction_the_angle_correction_is_good():
    robot_angle = 3*math.pi / 4

    vector_angle = 2 * math.pi / 3

    angle_correction = get_angle_correction(robot_angle, vector_angle)

    assert str(angle_correction)[:5] == "-0.26"


def test_given_2_3_when_get_angle_correction_the_angle_correction_is_good():
    robot_angle = 3*math.pi / 4

    vector_angle = 5 * math.pi / 4

    angle_correction = get_angle_correction(robot_angle, vector_angle)

    assert str(angle_correction)[:4] == "1.57"


def test_given_2_4_when_get_angle_correction_the_angle_correction_is_good():
    robot_angle = 3*math.pi / 4

    vector_angle = 7 * math.pi / 4

    angle_correction = get_angle_correction(robot_angle, vector_angle)

    assert str(angle_correction)[:4] == "3.14"


def test_given_3_1_when_get_angle_correction_the_angle_correction_is_good():
    robot_angle = 5*math.pi / 4

    vector_angle = 1 * math.pi / 4

    angle_correction = get_angle_correction(robot_angle, vector_angle)

    assert str(angle_correction)[:4] == "3.14"


def test_given_3_2_when_get_angle_correction_the_angle_correction_is_good():
    robot_angle = 5*math.pi / 4

    vector_angle = 3 * math.pi / 4

    angle_correction = get_angle_correction(robot_angle, vector_angle)

    assert str(angle_correction)[:5] == "-1.57"


def test_given_3_3_when_get_angle_correction_the_angle_correction_is_good():
    robot_angle = 5*math.pi / 4

    vector_angle = 4 * math.pi / 3

    angle_correction = get_angle_correction(robot_angle, vector_angle)

    assert str(angle_correction)[:4] == "0.26"


def test_given_3_4_when_get_angle_correction_the_angle_correction_is_good():
    robot_angle = 5*math.pi / 4

    vector_angle = 7 * math.pi / 4

    angle_correction = get_angle_correction(robot_angle, vector_angle)

    assert str(angle_correction)[:4] == "1.57"


def test_given_4_1_when_get_angle_correction_the_angle_correction_is_good():
    robot_angle = 7*math.pi / 4

    vector_angle = 1 * math.pi / 4

    angle_correction = get_angle_correction(robot_angle, vector_angle)

    assert str(angle_correction)[:4] == "1.57"


def test_given_4_2_when_get_angle_correction_the_angle_correction_is_good():
    robot_angle = 7*math.pi / 4

    vector_angle = 3 * math.pi / 4

    angle_correction = get_angle_correction(robot_angle, vector_angle)

    assert str(angle_correction)[:4] == "3.14"


def test_given_4_3_when_get_angle_correction_the_angle_correction_is_good():
    robot_angle = 7*math.pi / 4

    vector_angle = 5 * math.pi / 4

    angle_correction = get_angle_correction(robot_angle, vector_angle)

    assert str(angle_correction)[:5] == "-1.57"


def test_given_4_4_when_get_angle_correction_the_angle_correction_is_good():
    robot_angle = 7*math.pi / 4

    vector_angle = 5 * math.pi / 3

    angle_correction = get_angle_correction(robot_angle, vector_angle)

    assert str(angle_correction)[:5] == "-0.26"


def test_given_1_3_2_when_get_angle_correction_the_angle_correction_is_good():
    robot_angle = math.pi / 4

    vector_angle = -3 * math.pi / 4

    angle_correction = get_angle_correction(robot_angle, vector_angle)

    assert str(angle_correction)[:4] == "3.14"


def test_given_1_4_2_when_get_angle_correction_the_angle_correction_is_good():
    robot_angle = math.pi / 4

    vector_angle = -1 * math.pi / 4

    angle_correction = get_angle_correction(robot_angle, vector_angle)

    assert str(angle_correction)[:5] == "-1.57"


def test_given_2_3_2_when_get_angle_correction_the_angle_correction_is_good():
    robot_angle = 3*math.pi / 4

    vector_angle = -3 * math.pi / 4

    angle_correction = get_angle_correction(robot_angle, vector_angle)

    assert str(angle_correction)[:4] == "1.57"


def test_given_2_4_2_when_get_angle_correction_the_angle_correction_is_good():
    robot_angle = 3*math.pi / 4

    vector_angle = -1 * math.pi / 4

    angle_correction = get_angle_correction(robot_angle, vector_angle)

    assert str(angle_correction)[:4] == "3.14"


def test_given_3_1_2_when_get_angle_correction_the_angle_correction_is_good():
    robot_angle = -3*math.pi / 4

    vector_angle = 1 * math.pi / 4

    angle_correction = get_angle_correction(robot_angle, vector_angle)

    assert str(angle_correction)[:4] == "3.14"


def test_given_3_2_2_when_get_angle_correction_the_angle_correction_is_good():
    robot_angle = -3*math.pi / 4

    vector_angle = 3 * math.pi / 4

    angle_correction = get_angle_correction(robot_angle, vector_angle)

    assert str(angle_correction)[:5] == "-1.57"


def test_given_3_3_2_when_get_angle_correction_the_angle_correction_is_good():
    robot_angle = -3*math.pi / 4

    vector_angle = -2 * math.pi / 3

    angle_correction = get_angle_correction(robot_angle, vector_angle)

    assert str(angle_correction)[:4] == "0.26"


def test_given_3_4_2_when_get_angle_correction_the_angle_correction_is_good():
    robot_angle = -3*math.pi / 4

    vector_angle = -1 * math.pi / 4

    angle_correction = get_angle_correction(robot_angle, vector_angle)

    assert str(angle_correction)[:4] == "1.57"


def test_given_4_1_2_when_get_angle_correction_the_angle_correction_is_good():
    robot_angle = -1*math.pi / 4

    vector_angle = 1 * math.pi / 4

    angle_correction = get_angle_correction(robot_angle, vector_angle)

    assert str(angle_correction)[:4] == "1.57"


def test_given_4_2_2_when_get_angle_correction_the_angle_correction_is_good():
    robot_angle = -1*math.pi / 4

    vector_angle = 3 * math.pi / 4

    angle_correction = get_angle_correction(robot_angle, vector_angle)

    assert str(angle_correction)[:4] == "3.14"


def test_given_4_3_2_when_get_angle_correction_the_angle_correction_is_good():
    robot_angle = -1*math.pi / 4

    vector_angle = -3 * math.pi / 4

    angle_correction = get_angle_correction(robot_angle, vector_angle)

    assert str(angle_correction)[:5] == "-1.57"


def test_given_4_4_2_when_get_angle_correction_the_angle_correction_is_good():
    robot_angle = -1*math.pi / 4

    vector_angle = -1 * math.pi / 3

    angle_correction = get_angle_correction(robot_angle, vector_angle)

    assert str(angle_correction)[:5] == "-0.26"
