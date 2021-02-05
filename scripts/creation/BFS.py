from PathfinderAlgorithm import PathfinderAlgorithm
from PathNotFoundException import PathNotFoundException


class BFS(PathfinderAlgorithm):
    def find_path(self, start, end):
        queue = []
        visited = {start}

        queue.append([start])

        while queue:
            path = queue.pop(0)  # prendre une actual queue pour que ce soit plus efficace
            node = path[-1]

            if node == end:
                return path

            for neighbor in node.neighbors:
                if neighbor not in visited and neighbor.role == "empty":
                    queue.append(path + [neighbor])
                    visited.add(neighbor)

        raise PathNotFoundException()
