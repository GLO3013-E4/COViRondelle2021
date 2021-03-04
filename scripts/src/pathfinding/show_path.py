from PIL import Image

from scripts.src.pathfinding.pathfinder import Pathfinder
from scripts.src.pathfinding.pathfinding_algorithm_factory import PathfindingAlgorithmFactory
from scripts.src.pathfinding.map import Map


def get_path(node_size, algorithm, obstacles, start, end, pucks, image_path):
    # TODO: faire un pathfinder sans avoir besoin d'image (utiliser les points du top et du bas de la table?(murs de la table))

    image = Image.open(image_path)

    pathfinding_algorithm_factory = PathfindingAlgorithmFactory()
    pathfinding_algorithm = pathfinding_algorithm_factory.create(algorithm)
    board_map = Map(image, obstacles, pucks, start, end, node_size=node_size)
    board_map.render_map()
    pathfinder = Pathfinder(board_map, pathfinding_algorithm)

    pathfinder.find_square_matrix_path()
    return pathfinder.path
