from PIL import ImageDraw
from scripts.creation.TileRole import TileRole


class MapDrawer:
    def __init__(self, node_identifier_width, node_size, image):
        self.node_identifier_width = node_identifier_width
        self.node_size = node_size
        self.image = image
        self.width, self.height = self.image.size
        self.draw = ImageDraw.Draw(self.image)

    def draw_map(self, map, path):
        self.draw_board()

        for line in map.get_node_matrix():
            for node in line:
                if node.role is TileRole.OBSTACLE:
                    self.draw_obstacle(node)

                elif node.role is TileRole.CUSHION:
                    self.draw_cushion(node)

                elif node.role is TileRole.PUCK:
                    self.draw_puck(node)

                elif node.role is TileRole.START:
                    self.draw_start_node(node)

                elif node.role is TileRole.END:
                    self.draw_end_node(node)

                elif node.role is TileRole.EMPTY:
                    pass

                else:
                    raise  # TODO:

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
        x,y = node.pixel_coordinates_center
        self.draw.rectangle([(x - self.node_identifier_width, y - self.node_identifier_width),
                             (x + self.node_identifier_width, y + self.node_identifier_width)], fill=120)

    def draw_path(self, path):
        for node in path:
            x,y = node.pixel_coordinates_center
            self.draw.rectangle([(x - self.node_identifier_width, y - self.node_identifier_width),
                                 (x + self.node_identifier_width, y + self.node_identifier_width)], outline=300)

    def get_image(self):
        return self.image
