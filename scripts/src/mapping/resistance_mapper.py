class ResistanceMapper:
    """Class used to map resistances to colors"""
    def __init__(self):
        # TODO: est-ce les couleurs devraient être dans un enum ou
        #  ils devraient être des couleurs de opencv/pillow/autre?
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
        }

    def find_colors(self, resistance):
        return [
            self.number_to_color[resistance.get_first_digit()],
            self.number_to_color[resistance.get_second_digit()],
            self.number_to_color[resistance.get_exponent()]
        ]
