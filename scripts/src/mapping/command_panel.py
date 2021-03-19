from scripts.src.mapping.corner import Corner
from scripts.src.mapping.letter import Letter
from scripts.src.mapping.resistance import Resistance
from scripts.src.mapping.resistance_mapper import ResistanceMapper


class CommandPanel:
    def __init__(self):
        self.coordinates = None
        self.resistance = None
        self.mapped_letters = None
        self.resistance_mapper = ResistanceMapper()

    def set_coordinates(self, coordinates):
        self.coordinates = coordinates

    def set_resistance(self, resistance):
        self.resistance = Resistance(resistance)

    def set_mapped_letters(self, mapped_letters):
        self.mapped_letters = mapped_letters

    def can_find_goals(self):
        return self.coordinates and self.resistance and self.coordinates

    def get_goals(self):
        if not self.can_find_goals():
            raise Exception("Either the coordinates, the resistance"
                            "or the mapped_letters are set to None.")
        first_corner_letter = self.find_first_corner_letter()
        second_corner_letter = first_corner_letter.get_next_letter()
        third_corner_letter = second_corner_letter.get_next_letter()

        colors = self.resistance_mapper.find_colors(self.resistance)
        corners = self.decode_corners()

        for i, color in enumerate(colors):
            corners[i].set_color(color)

        first_corner = [corner for corner in corners if corner.letter is first_corner_letter][0]
        second_corner = [corner for corner in corners if corner.letter is second_corner_letter][0]
        third_corner = [corner for corner in corners if corner.letter is third_corner_letter][0]
        return [first_corner, second_corner, third_corner]

    def find_first_corner_letter(self):
        first_digit = self.resistance.get_first_digit()
        index = first_digit - 1
        return Letter[self.mapped_letters[index]]

    def decode_corners(self):
        position_A = self.decode_corner_A(self.coordinates)
        position_B = self.decode_corner_B(self.coordinates)
        position_C = self.decode_corner_C(self.coordinates)
        position_D = self.decode_corner_D(self.coordinates)
        return [
            Corner(Letter.A, position_A),
            Corner(Letter.B, position_B),
            Corner(Letter.C, position_C),
            Corner(Letter.D, position_D)
        ]

    def decode_corner_A(self, coordinates):
        coordinates = sorted(coordinates, key=lambda x: x[1])
        potential_coordinates = coordinates[:2]
        potential_coordinates = sorted(potential_coordinates, key=lambda x: x[0])
        return potential_coordinates[1]

    def decode_corner_B(self, coordinates):
        coordinates = sorted(coordinates, key=lambda x: x[1])
        potential_coordinates = coordinates[2:]
        potential_coordinates = sorted(potential_coordinates, key=lambda x: x[0])
        return potential_coordinates[1]

    def decode_corner_C(self, coordinates):
        coordinates = sorted(coordinates, key=lambda x: x[1])
        potential_coordinates = coordinates[2:]
        potential_coordinates = sorted(potential_coordinates, key=lambda x: x[0])
        return potential_coordinates[0]

    def decode_corner_D(self, coordinates):
        coordinates = sorted(coordinates, key=lambda x: x[1])
        potential_coordinates = coordinates[:2]
        potential_coordinates = sorted(potential_coordinates, key=lambda x: x[0])
        return potential_coordinates[0]
