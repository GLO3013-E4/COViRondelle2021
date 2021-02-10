from scripts.creation.BreadthFirstSearch import BreadthFirstSearch


class PathfindingAlgorithmFactory:
    def create(self, algorithm):
        if algorithm == "BreadthFirstSearch":
            return BreadthFirstSearch()
        else:
            raise Exception("Chosen pathfinding algorithm has not yet been implemented.")
