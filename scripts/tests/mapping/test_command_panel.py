import pytest

from scripts.src.mapping.command_panel import CommandPanel
from scripts.src.mapping.letter import Letter


class TestCommandPanel:
    def setup_method(self):
        self.command_panel = CommandPanel()

    def test_given_multiple_coordinates_when_decode_corners_then_associate_corners_correctly(self):
        coordinates = [(0, 0), (0, 1), (1, 0), (1, 1)]
        self.command_panel.set_coordinates(coordinates)

        corners = self.command_panel.decode_corners()
        corner_A = [corner for corner in corners if corner.letter is Letter.A][0]
        corner_B = [corner for corner in corners if corner.letter is Letter.B][0]
        corner_C = [corner for corner in corners if corner.letter is Letter.C][0]
        corner_D = [corner for corner in corners if corner.letter is Letter.D][0]

        assert corner_A.position == (1, 0)
        assert corner_B.position == (1, 1)
        assert corner_C.position == (0, 1)
        assert corner_D.position == (0, 0)

    def test_given_multiple_coordinates_when_decode_corner_a_then_return_correct_coordinates(self):
        coordinates = [(0, 0), (0, 1), (1, 0), (1, 1)]
        self.command_panel.set_coordinates(coordinates)

        corner_a = self.command_panel.decode_corner_A(coordinates)

        assert corner_a == (1, 0)

    def test_given_multiple_coordinates_when_decode_corner_b_then_return_correct_coordinates(self):
        coordinates = [(0, 0), (0, 1), (1, 0), (1, 1)]
        self.command_panel.set_coordinates(coordinates)

        corner_b = self.command_panel.decode_corner_B(coordinates)

        assert corner_b == (1, 1)

    def test_given_multiple_coordinates_when_decode_corner_c_then_return_correct_coordinates(self):
        coordinates = [(0, 0), (0, 1), (1, 0), (1, 1)]
        self.command_panel.set_coordinates(coordinates)

        corner_c = self.command_panel.decode_corner_C(coordinates)

        assert corner_c == (0, 1)

    def test_given_multiple_coordinates_when_decode_corner_d_then_return_correct_coordinates(self):
        coordinates = [(0, 0), (0, 1), (1, 0), (1, 1)]
        self.command_panel.set_coordinates(coordinates)

        corner_d = self.command_panel.decode_corner_D(coordinates)

        assert corner_d == (0, 0)

    def test_when_find_first_corner_letter_1_then_return_correct_letter(self):
        mapped_letters = ["A", "B", "B", "D", "B", "B", "B", "B", "B"]
        resistance = 1234
        self.command_panel.set_mapped_letters(mapped_letters)
        self.command_panel.set_resistance(resistance)

        letter = self.command_panel.find_first_corner_letter()

        assert letter is Letter.A

    def test_when_find_first_corner_letter_2_then_return_correct_letter(self):
        mapped_letters = ["A", "B", "B", "D", "B", "B", "B", "B", "B"]
        resistance = 4567
        self.command_panel.set_mapped_letters(mapped_letters)
        self.command_panel.set_resistance(resistance)

        letter = self.command_panel.find_first_corner_letter()

        assert letter is Letter.D

    def test_given_corners_resistance_and_letters_when_get_goals_then_return_correct_corners(self):
        coordinates = [(0, 0), (0, 1), (1, 0), (1, 1)]
        resistance = 1234
        mapped_letters = ["A", "B", "B", "D", "B", "B", "B", "B", "B"]
        self.command_panel.set_mapped_letters(mapped_letters)
        self.command_panel.set_coordinates(coordinates)
        self.command_panel.set_resistance(resistance)

        goals = self.command_panel.get_goals()
        first_corner, second_corner, third_corner = goals

        assert first_corner.letter is Letter.A
        assert second_corner.letter is Letter.B
        assert third_corner.letter is Letter.C

    def test_given_image_2_when_get_goals_then_return_correct_corners(self):
        coordinates = [(0, 0), (0, 1), (1, 0), (1, 1)]
        resistance = 4567
        mapped_letters = ["A", "B", "B", "D", "B", "B", "B", "B", "B"]
        self.command_panel.set_mapped_letters(mapped_letters)
        self.command_panel.set_coordinates(coordinates)
        self.command_panel.set_resistance(resistance)

        goals = self.command_panel.get_goals()
        first_corner, second_corner, third_corner = goals

        assert first_corner.letter is Letter.D
        assert second_corner.letter is Letter.A
        assert third_corner.letter is Letter.B

    def test_given_no_set_coordinates_when_get_goals_then_throws(self):
        resistance = 4567
        mapped_letters = ["A", "B", "B", "D", "B", "B", "B", "B", "B"]
        self.command_panel.set_mapped_letters(mapped_letters)
        self.command_panel.set_resistance(resistance)

        with pytest.raises(Exception):
            self.command_panel.get_goals()

    def test_given_no_set_resistance_when_get_goals_then_throws(self):
        coordinates = [(0, 0), (0, 1), (1, 0), (1, 1)]
        mapped_letters = ["A", "B", "B", "D", "B", "B", "B", "B", "B"]
        self.command_panel.set_mapped_letters(mapped_letters)
        self.command_panel.set_coordinates(coordinates)

        with pytest.raises(Exception):
            self.command_panel.get_goals()

    def test_given_no_set_mapped_letters_when_get_goals_then_throws(self):
        coordinates = [(0, 0), (0, 1), (1, 0), (1, 1)]
        resistance = 4567
        self.command_panel.set_coordinates(coordinates)
        self.command_panel.set_resistance(resistance)

        with pytest.raises(Exception):
            self.command_panel.get_goals()
