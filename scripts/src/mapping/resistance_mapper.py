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
