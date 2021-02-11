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

    def _draw_line(self, x1, y1, x2, y2, color):
        self.draw.line((x1, y1, x2, y2), fill=color)

    def _draw_rectangle(self, node, color, outline=None):
        x, y = node.pixel_coordinates_center
        self.draw.rectangle([(x - self.node_identifier_width, y - self.node_identifier_width),
                             (x + self.node_identifier_width, y + self.node_identifier_width)], fill=color, outline=outline)

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
            x1 = i * self.node_size
            y1 = 0
            x2 = i * self.node_size
            y2 = self.height
            color = 128

            self._draw_line(x1, y1, x2, y2, color)

    def draw_horizontal_lines(self):
        for i in range((self.height // self.node_size) + 1):
            x1 = 0
            y1 = i * self.node_size
            x2 = self.width
            y2 = i * self.node_size
            color = 128

            self._draw_line(x1, y1, x2, y2, color)

    def draw_cushion(self, node):
        color = (0, 255, 0)
        self._draw_rectangle(node, color)

    def draw_obstacle(self, node):
        color = (255, 255, 255)
        self._draw_rectangle(node, color)

    def draw_puck(self, node):
        color = (0, 0, 255)
        self._draw_rectangle(node, color)

    def draw_start_node(self, node):
        color = 200
        self._draw_rectangle(node, color)

    def draw_end_node(self, node):
        color = 120
        self._draw_rectangle(node, color)

    def draw_path(self, path):
        for node in path:
            color = (0, 122, 9)
            outline = 300

            self._draw_rectangle(node, color, outline)

    def get_image(self):
        return self.image
