from math import atan2, pi


class PositionCalculator:

    def calculate_angle_between_two_position(self, position_one, position_two):
        x_difference = self.calculate_difference_of_x(position_one, position_two)
        y_difference = self.calculate_difference_of_y(position_one, position_two)
        angle = atan2(y_difference, x_difference)
        return self.convert_negative_to_positive(angle)

    def calculate_difference_of_x(self, position_one, position_two):
        return position_two[0] - position_one[0]

    def calculate_difference_of_y(self, position_one, position_two):
        return -(position_two[1] - position_one[1])

    def convert_negative_to_positive(self, angle):
        if angle < 0:
            angle += 2*pi
        return angle