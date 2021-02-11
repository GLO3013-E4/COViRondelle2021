import math


class Resistance:
    def __init__(self, resistance):
        self.resistance = resistance
        self.resistance = self.round()
        self.exponent = self.find_exponent()
        self.first_digit, self.second_digit = self.find_first_digits()

    def round(self):
        return (self.resistance + 99) // 100 * 100

    def find_exponent(self):
        return int(math.log10(self.resistance) - 1)

    def find_first_digits(self):
        resistance = self.resistance * 100
        return [int(digit) for digit in str(resistance)[:2]]

    def get_exponent(self):
        return self.exponent

    def get_first_digit(self):
        return self.first_digit

    def get_second_digit(self):
        return self.second_digit
