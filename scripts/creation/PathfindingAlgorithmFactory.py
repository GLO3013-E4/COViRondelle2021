from scripts.creation.BFS import BFS


class PathfindingAlgorithmFactory:
    def create(self, algorithm):
        if algorithm == "BFS":
            return BFS()
        else:
            raise Exception("Chosen pathfinding has not yet been implemented.")
