class ObstaclePosition:
    def generate_obstacle_dict(self, top_right, top_left, bottom_right, bottom_left, obstacle_id):
        obstacle_report = {
            f"obstacle {obstacle_id}" : {
            "top_right": top_right,
            "top_left": top_left,
            "bottom_right": bottom_right,
            "bottom_left": bottom_left
            }
        }
        return obstacle_report

    def generate_center_position(self, bottom_right_position, top_left_position):
        center_x = int((top_left_position[0] + bottom_right_position[0]) / 2.0)
        center_y = int((top_left_position[1] + bottom_right_position[1]) / 2.0)
        return center_x, center_y