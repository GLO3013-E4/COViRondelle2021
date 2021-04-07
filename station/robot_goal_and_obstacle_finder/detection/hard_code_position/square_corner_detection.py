class SquareCornerDetection:

    def position_of_corner_a(self):
        return [600, 226]

    def position_of_corner_b(self):
        return [608, 645]

    def position_of_corner_c(self):
        return [185, 636]

    def position_of_corner_d(self):
        return [181, 226]

    def center_position_of_square(self):
        return [397, 430]

    def get_dictionary_of_square_position(self):
        return {
            "corner_A": self.position_of_corner_a(),
            "corner_B": self.position_of_corner_b(),
            "corner_C": self.position_of_corner_c(),
            "corner_D": self.position_of_corner_d()
        }