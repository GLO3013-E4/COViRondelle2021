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

    def detect_square_position(self):
        position_a = self.position_of_corner_a()
        position_b = self.position_of_corner_b()
        position_c = self.position_of_corner_c()
        position_d = self.position_of_corner_d()
        center = self.center_position_of_square()

        return self.get_dictionary_of_square_position(position_a=position_a,
                                                      position_b=position_b,
                                                      position_c=position_c,
                                                      position_d=position_d,
                                                      center=center)

    def get_dictionary_of_square_position(self, position_a, position_b, position_c, position_d, center):
        return {
            "corner_A": position_a,
            "corner_B": position_b,
            "corner_C": position_c,
            "corner_D": position_d,
            "center": center
        }
