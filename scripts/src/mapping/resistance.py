import math


class Resistance:
    """Class used to encapsulate resistances and then allow us to be able to
    extract useful information for the resistance color code sheet"""
    def __init__(self, resistance):
        self.resistance = resistance

    def round(self):
        return round(self.resistance / 100) * 100

    def get_exponent(self):
        resistance = self.round()
        return int(math.log10(resistance) - 1)

    def find_first_digits(self):
        resistance = self.round()
        resistance = resistance * 100
        return [int(digit) for digit in str(resistance)[:2]]

    def get_first_digit(self):
        return self.find_first_digits()[0]

    def get_second_digit(self):
        return self.find_first_digits()[1]
