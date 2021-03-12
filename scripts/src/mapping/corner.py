from scripts.src.mapping.letter import Letter


class Corner:
    def __init__(self, letter):
        self.next_letter = {
            Letter.A: Letter.B,
            Letter.B: Letter.C,
            Letter.C: Letter.D,
            Letter.D: Letter.A
        }

        self.letter = letter

    def get_next(self):
        return self.next_letter[self.letter]
