import math

from mapping.resistance_value import ResistanceValue


class Resistance:
    """Class used to encapsulate resistances and then allow us to be able to
    extract useful information for the resistance color code sheet"""

    def __init__(self, resistance):
        self.resistance = resistance

    def round(self):
        log10 = math.floor(math.log10(self.resistance))
        resistance_values = [resistance[0]*(pow(10, log10)) for resistance in ResistanceValue().RESISTANCE_VALUES()]

        distances = [(i, abs(self.resistance - model_resistance)) for i, model_resistance in enumerate(resistance_values)]

        closest_index = min(distances, key=lambda x: x[1])[0]
        closest_resistance = resistance_values[closest_index]
        return closest_resistance

    def get_exponent(self):
        resistance = self.round()
        return math.floor(math.log10(resistance) - 1)

    def find_first_digits(self):
        resistance = self.round()
        resistance = resistance * 100
        return [int(digit) for digit in str(resistance)[:2]]

    def get_first_digit(self):
        return self.find_first_digits()[0]

    def get_second_digit(self):
        return self.find_first_digits()[1]
