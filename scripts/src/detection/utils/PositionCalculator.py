from math import atan2, degrees

class PositionCalculator:

    @staticmethod
    def calculate_angle_between_two_position(position_one, position_two):
        x_difference = PositionCalculator.calculate_difference_of_x(position_one, position_two)
        y_difference = PositionCalculator.calculate_difference_of_y(position_one, position_two)
        return degrees(atan2(y_difference, x_difference))

    @staticmethod
    def calculate_difference_of_x(position_one, position_two):
        return position_two.get_position_x() - position_one.get_position_x()

    @staticmethod
    def calculate_difference_of_y(position_one, position_two):
        return position_two.get_position_y() - position_one.get_position_y()