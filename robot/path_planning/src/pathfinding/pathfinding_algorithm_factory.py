"""Factory used to create and control the instantiation of path-finding algorithms"""

from scripts.src.pathfinding.breadth_first_search import BreadthFirstSearch


class PathfindingAlgorithmFactory:
    """Factory used to create and control the instantiation of path-finding algorithms"""
    @staticmethod
    def create(algorithm):
        """Create and controls the instantiation of path-finding algorithms"""
        if algorithm == "BreadthFirstSearch":
            return BreadthFirstSearch()
        raise Exception("Chosen pathfinding algorithm has not yet been implemented.")
