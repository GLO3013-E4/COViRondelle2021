# résistances entre 10^2 et 10^6 ohms

from scripts.src.mapping.resistance import Resistance
from scripts.src.mapping.resistance_mapper import ResistanceMapper

if __name__ == '__main__':
    resistance_mapper = ResistanceMapper()
    for res in [3553, 3933, 9233, 9933, 35604]:
        resistance = Resistance(res)

        print(resistance_mapper.find_colors(resistance))
