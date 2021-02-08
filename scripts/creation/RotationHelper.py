import math


class RotationHelper:
    def find_angle_to_turn(self, start, gripper, end, final_path_node):
        x1, y1 = start
        x2, y2 = gripper
        initial_angle = math.atan2(y2 - y1, x2 - x1) * 180 / math.pi

        x1, y1 = end
        x2, y2 = final_path_node
        final_angle = math.atan2(y2 - y1, x2 - x1) * 180 / math.pi

        target = 180 - final_angle * 180 / math.pi
        rotation = target - initial_angle * 180 / math.pi
        return rotation
