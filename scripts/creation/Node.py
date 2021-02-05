class Node:
    def __init__(self, matrix_center, pixel_coordinates_center, width, height):
        self.matrix_center = matrix_center
        self.pixel_coordinates_center = pixel_coordinates_center
        self.width = width
        self.height = height
        self.neighbors = []
        self.role = "empty"