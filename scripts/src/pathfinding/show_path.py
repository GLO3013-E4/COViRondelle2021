from scripts.src.pathfinding.pathfinder import Pathfinder
from scripts.src.pathfinding.pathfinding_algorithm_factory import PathfindingAlgorithmFactory
from scripts.src.pathfinding.map import Map
from scripts.src.util.time_it import time_it


def get_path(node_size, algorithm, obstacles, start, end, pucks, image_width, image_height):
    pathfinding_algorithm_factory = PathfindingAlgorithmFactory()
    pathfinding_algorithm = pathfinding_algorithm_factory.create(algorithm)
    board_map = Map(image_width, image_height, obstacles, pucks, start, end, node_size=node_size)
    board_map.render_map()
    pathfinder = Pathfinder(board_map, pathfinding_algorithm)

    pathfinder.find_square_matrix_path()
    return pathfinder.path


@time_it
def get_path_and_map(node_size, algorithm, obstacles, start, end, pucks, image_width, image_height):
    pathfinding_algorithm_factory = PathfindingAlgorithmFactory()
    pathfinding_algorithm = pathfinding_algorithm_factory.create(algorithm)
    board_map = Map(image_width, image_height, obstacles, pucks, start, end, node_size=node_size)
    board_map.render_map()
    pathfinder = Pathfinder(board_map, pathfinding_algorithm)

    pathfinder.find_square_matrix_path()

    return pathfinder.path, board_map
