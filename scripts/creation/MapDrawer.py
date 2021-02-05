#TODO: décider draw au fur et à mésure ou tout d'un coup?
#TODO: live ça serait tout d'un coup

from PIL import ImageDraw


class MapDrawer:
    def __init__(self, node_identifier_width, node_size, image):
        self.node_identifier_width = node_identifier_width
        self.node_size = node_size
        self.image = image
        self.image_height, self.image_width = self.image.size
        self.draw = ImageDraw.Draw(self.image)

    def draw_map(self, map, path):
        self.draw_board()

        for line in map.get_node_matrix():
            for node in line:
                if node.role == "obstacle":
                    self.draw_obstacle(node)

                elif node.role == "obstacle_cushion":
                    self.draw_cushion(node)

                elif node.role == "puck":
                    self.draw_puck(node)

                elif node.role == "start":
                    #TODO:add role start + end juste à cause du drawer?
                    self.draw_start_node(node)

                elif node.role == "end":
                    #TODO:add role start + end juste à cause du drawer?
                    self.draw_end_node(node)

                elif node.role == "empty":
                    pass

                else:
                    raise  # TODO:

        self.draw_path(draw, path)

    def draw_board(self):
        self.draw_vertical_lines(draw)
        self.draw_horizontal_lines(draw)

    def draw_vertical_lines(self):
        for i in range((self.image_width // self.node_size) + 1):
            self.draw.line((i * self.node_size, 0, i * self.node_size, self.image_height), fill=128)

    def draw_horizontal_lines(self):
        for i in range((self.image_height // self.node_size) + 1):
            self.draw.line((0, i * self.node_size, self.image_width, i * self.node_size), fill=128)

    def draw_cushion(self, node):
        y1, x1 = node.pixel_coordinates_center
        self.draw.rectangle([(x1 - self.node_identifier_width, y1 - self.node_identifier_width),
                        (x1 + self.node_identifier_width, y1 + self.node_identifier_width)], fill=(0, 255, 0))

    def draw_obstacle(self, node):
        y, x = node.pixel_coordinates_center
        self.draw.rectangle([(x - self.node_identifier_width, y - self.node_identifier_width),
                        (x + self.node_identifier_width, y + self.node_identifier_width)], fill=(255, 255, 255))

    def draw_puck(self, node):
        pass

    def draw_start_node(self, node):
        y, x = node.pixel_coordinates_center
        self.draw.rectangle([(x - self.node_identifier_width, y - self.node_identifier_width),
                        (x + self.node_identifier_width, y + self.node_identifier_width)], fill=200)

    def draw_end_node(self, node):
        y, x = node.pixel_coordinates_center
        self.draw.rectangle([(x - self.node_identifier_width, y - self.node_identifier_width),
                        (x + self.node_identifier_width, y + self.node_identifier_width)], fill=120)

    def draw_path(self, path):
        for node in path:
            y, x = node.pixel_coordinates_center
            self.draw.rectangle([(x - self.node_identifier_width, y - self.node_identifier_width),
                            (x + self.node_identifier_width, y + self.node_identifier_width)], outline=300)
