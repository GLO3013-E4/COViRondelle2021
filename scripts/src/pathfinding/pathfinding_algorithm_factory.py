"""Factory used to create and control the instantiation of path-finding algorithms"""

from scripts.src.pathfinding.breadth_first_search import BreadthFirstSearch
from scripts.src.pathfinding.a_star import AStar


class PathfindingAlgorithmFactory:
    """Factory used to create and control the instantiation of path-finding algorithms"""
    @staticmethod
    def create(algorithm):
        """Create and controls the instantiation of path-finding algorithms"""
        if algorithm == "BreadthFirstSearch":
            return BreadthFirstSearch()
        elif algorithm == "A*":
            return AStar()
        raise Exception("Chosen pathfinding algorithm has not yet been implemented.")
