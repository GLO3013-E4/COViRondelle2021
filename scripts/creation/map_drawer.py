from PIL import ImageDraw

from scripts.creation.tile_role import TileRole


class MapDrawer:
    def __init__(self, node_identifier_width, node_size, image):
        self.node_identifier_width = node_identifier_width
        self.node_size = node_size
        self.image = image
        self.width, self.height = self.image.size
        self.draw = ImageDraw.Draw(self.image)
        self.role_to_drawing_function = self.get_role_to_drawing_function()

    def get_role_to_drawing_function(self):
        return {
            TileRole.OBSTACLE: self.draw_obstacle,
            TileRole.CUSHION: self.draw_cushion,
            TileRole.PUCK: self.draw_puck,
            TileRole.START: self.draw_start_node,
            TileRole.END: self.draw_end_node,
            TileRole.EMPTY: lambda x: None
        }

    def draw_map(self, map, path):
        self.draw_board()

        for line in map.get_node_matrix():
            for node in line:
                drawing_function = self.role_to_drawing_function[node.role]
                drawing_function(node)

        self.draw_path(path)

    def draw_board(self):
        self.draw_vertical_lines()
        self.draw_horizontal_lines()

    def draw_vertical_lines(self):
        for i in range((self.width // self.node_size) + 1):
            self.draw.line((i * self.node_size, 0, i * self.node_size, self.height), fill=128)

    def draw_horizontal_lines(self):
        for i in range((self.height // self.node_size) + 1):
            self.draw.line((0, i * self.node_size, self.width, i * self.node_size), fill=128)

    def draw_cushion(self, node):
        x, y = node.pixel_coordinates_center
        self.draw.rectangle([(x - self.node_identifier_width, y - self.node_identifier_width),
                             (x + self.node_identifier_width, y + self.node_identifier_width)], fill=(0, 255, 0))

    def draw_obstacle(self, node):
        x, y = node.pixel_coordinates_center
        self.draw.rectangle([(x - self.node_identifier_width, y - self.node_identifier_width),
                             (x + self.node_identifier_width, y + self.node_identifier_width)], fill=(255, 255, 255))

    def draw_puck(self, node):
        x, y = node.pixel_coordinates_center
        self.draw.rectangle([(x - self.node_identifier_width, y - self.node_identifier_width),
                             (x + self.node_identifier_width, y + self.node_identifier_width)], fill=(0, 0, 255))

    def draw_start_node(self, node):
        x, y = node.pixel_coordinates_center
        self.draw.rectangle([(x - self.node_identifier_width, y - self.node_identifier_width),
                             (x + self.node_identifier_width, y + self.node_identifier_width)], fill=200)

    def draw_end_node(self, node):
        x, y = node.pixel_coordinates_center
        self.draw.rectangle([(x - self.node_identifier_width, y - self.node_identifier_width),
                             (x + self.node_identifier_width, y + self.node_identifier_width)], fill=120)

    def draw_path(self, path):
        for node in path:
            x, y = node.pixel_coordinates_center
            self.draw.rectangle([(x - self.node_identifier_width, y - self.node_identifier_width),
                                 (x + self.node_identifier_width, y + self.node_identifier_width)], outline=300)

    def get_image(self):
        return self.image
