from BFS import BFS


class PathfindingAlgorithmFactory:
    def create(self, algorithm):
        if algorithm == "BFS":
            return BFS()
        else:
            raise  # TODO:
