from PathfinderAlgorithm import PathfinderAlgorithm
from PathNotFoundException import PathNotFoundException
from TileRole import TileRole
from collections import deque


class BFS(PathfinderAlgorithm):
    def find_path(self, start):
        queue = deque()
        visited = {start}

        queue.append([start])

        while queue:
            path = queue.popleft()
            node = path[-1]

            if node.role is TileRole.END:
                return path

            for neighbor, angle in node.neighbors:
                if neighbor not in visited and (neighbor.role is TileRole.EMPTY or neighbor.role is TileRole.END or neighbor.role is TileRole.START):
                    queue.append(path + [neighbor])
                    visited.add(neighbor)

        raise PathNotFoundException()
