# résistances entre 10^2 et 10^6 ohms
import math


class Resistance:
    def __init__(self, resistance):
        self.ohms = self.round(resistance)
        self.exponent = self.find_exponent(resistance)
        self.first_digit, self.second_digit = self.find_first_digits(resistance)

    def round(self, resistance):
        return (resistance + 99) // 100 * 100

    def find_exponent(self, resistance):
        return int(math.log10(resistance) - 1)

    def find_first_digits(self, resistance):
        resistance = resistance * 100
        return [int(digit) for digit in str(resistance)[:2]]

    def get_exponent(self):
        return self.exponent

    def get_first_digit(self):
        return self.first_digit

    def get_second_digit(self):
        return self.second_digit


class ResistanceMapper:
    def __init__(self):
        self.color_to_number = {
            'noir': 0,
            'marron': 1,
            'rouge': 2,
            'orange': 3,
            'jaune': 4,
            'vert': 5,
            'bleu': 6,
            'violet': 7,
            'gris': 8,
            'blanc': 9
            # or
            # argent
            # (absent)
        }

        self.number_to_color = {
            0: 'noir',
            1: 'marron',
            2: 'rouge',
            3: 'orange',
            4: 'jaune',
            5: 'vert',
            6: 'bleu',
            7: 'violet',
            8: 'gris',
            9: 'blanc'
            #
            #
            #
        }

    def find_colors(self, resistance):
        return [
            self.number_to_color[resistance.get_first_digit()],
            self.number_to_color[resistance.get_second_digit()],
            self.number_to_color[resistance.get_exponent()]
        ]

#3553 -> 3600 ?
#3933 -> 4000 ?
#9233 -> 9300 ?
#9933 -> 10000 ?


if __name__ == '__main__':
    resistance_mapper = ResistanceMapper()
    for res in [3553, 3933, 9233, 9933, 35604]:
        resistance = Resistance(res)

        print(resistance_mapper.find_colors(resistance))
