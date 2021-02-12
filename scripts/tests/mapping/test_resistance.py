from scripts.src.mapping.resistance import Resistance


class TestResistance:
    """Test Resistance class"""
    @classmethod
    def setup_class(cls):
        cls.A_RESISTANCE_1 = 3553
        cls.A_RESISTANCE_2 = 3933
        cls.A_RESISTANCE_3 = 9233
        cls.A_RESISTANCE_4 = 9933

        cls.EXPECTED_RESISTANCE_1 = 3600
        cls.EXPECTED_RESISTANCE_2 = 4000
        cls.EXPECTED_RESISTANCE_3 = 9300
        cls.EXPECTED_RESISTANCE_4 = 10000
        cls.EXPECTED_FIRST_DIGIT_1 = 3
        cls.EXPECTED_FIRST_DIGIT_2 = 4
        cls.EXPECTED_FIRST_DIGIT_3 = 9
        cls.EXPECTED_FIRST_DIGIT_4 = 1
        cls.EXPECTED_SECOND_DIGIT_1 = 6
        cls.EXPECTED_SECOND_DIGIT_2 = 0
        cls.EXPECTED_SECOND_DIGIT_3 = 3
        cls.EXPECTED_SECOND_DIGIT_4 = 0
        cls.EXPECTED_EXPONENT_1 = 2
        cls.EXPECTED_EXPONENT_2 = 2
        cls.EXPECTED_EXPONENT_3 = 2
        cls.EXPECTED_EXPONENT_4 = 3

    def test_when_round_then_resistance_is_as_expected_1(self):
        resistance = Resistance(self.A_RESISTANCE_1)
        rounded_resistance = resistance.round()
        assert rounded_resistance == self.EXPECTED_RESISTANCE_1

    def test_when_round_then_resistance_is_as_expected_2(self):
        resistance = Resistance(self.A_RESISTANCE_2)
        rounded_resistance = resistance.round()
        assert rounded_resistance == self.EXPECTED_RESISTANCE_2

    def test_when_round_then_resistance_is_as_expected_3(self):
        resistance = Resistance(self.A_RESISTANCE_3)
        rounded_resistance = resistance.round()
        assert rounded_resistance == self.EXPECTED_RESISTANCE_3

    def test_when_round_then_resistance_is_as_expected_4(self):
        resistance = Resistance(self.A_RESISTANCE_4)
        rounded_resistance = resistance.round()
        assert rounded_resistance == self.EXPECTED_RESISTANCE_4

    def test_when_find_first_digits_then_digits_are_as_expected_1(self):
        resistance = Resistance(self.A_RESISTANCE_1)
        first_digit, second_digit = resistance.find_first_digits()
        assert first_digit == self.EXPECTED_FIRST_DIGIT_1
        assert second_digit == self.EXPECTED_SECOND_DIGIT_1

    def test_when_find_first_digits_then_digits_are_as_expected_2(self):
        resistance = Resistance(self.A_RESISTANCE_2)
        first_digit, second_digit = resistance.find_first_digits()
        assert first_digit == self.EXPECTED_FIRST_DIGIT_2
        assert second_digit == self.EXPECTED_SECOND_DIGIT_2

    def test_when_find_first_digits_then_digits_are_as_expected_3(self):
        resistance = Resistance(self.A_RESISTANCE_3)
        first_digit, second_digit = resistance.find_first_digits()
        assert first_digit == self.EXPECTED_FIRST_DIGIT_3
        assert second_digit == self.EXPECTED_SECOND_DIGIT_3

    def test_when_find_first_digits_then_digits_are_as_expected_4(self):
        resistance = Resistance(self.A_RESISTANCE_4)
        first_digit, second_digit = resistance.find_first_digits()
        assert first_digit == self.EXPECTED_FIRST_DIGIT_4
        assert second_digit == self.EXPECTED_SECOND_DIGIT_4

    def test_when_find_exponent_then_exponent_is_as_expected_1(self):
        resistance = Resistance(self.A_RESISTANCE_1)
        exponent = resistance.get_exponent()
        assert exponent == self.EXPECTED_EXPONENT_1

    def test_when_find_exponent_then_exponent_is_as_expected_2(self):
        resistance = Resistance(self.A_RESISTANCE_2)
        exponent = resistance.get_exponent()
        assert exponent == self.EXPECTED_EXPONENT_2

    def test_when_find_exponent_then_exponent_is_as_expected_3(self):
        resistance = Resistance(self.A_RESISTANCE_3)
        exponent = resistance.get_exponent()
        assert exponent == self.EXPECTED_EXPONENT_3

    def test_when_find_exponent_then_exponent_is_as_expected_4(self):
        resistance = Resistance(self.A_RESISTANCE_4)
        exponent = resistance.get_exponent()
        assert exponent == self.EXPECTED_EXPONENT_4
