from scripts.creation.breadth_first_search import BreadthFirstSearch


class PathfindingAlgorithmFactory:
    def create(self, algorithm):
        if algorithm == "BreadthFirstSearch":
            return BreadthFirstSearch()
        else:
            raise Exception("Chosen pathfinding algorithm has not yet been implemented.")
