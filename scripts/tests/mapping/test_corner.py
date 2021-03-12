from scripts.src.mapping.corner import Corner
from scripts.src.mapping.letter import Letter


class TestCorner:
    def setup_method(self):
        pass

    def test_given_letter_a_when_get_next_then_get_letter_b(self):
        letter = Corner(Letter.A)

        next_letter = letter.get_next()

        assert next_letter is Letter.B

    def test_given_letter_b_when_get_next_then_get_letter_c(self):
        letter = Corner(Letter.B)

        next_letter = letter.get_next()

        assert next_letter is Letter.C

    def test_given_letter_c_when_get_next_then_get_letter_d(self):
        letter = Corner(Letter.C)

        next_letter = letter.get_next()

        assert next_letter is Letter.D

    def test_given_letter_d_when_get_next_then_get_letter_a(self):
        letter = Corner(Letter.D)

        next_letter = letter.get_next()

        assert next_letter is Letter.A
