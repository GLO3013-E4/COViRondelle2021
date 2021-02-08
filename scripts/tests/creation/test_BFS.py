import pytest

from scripts.creation.Node import Node
from scripts.creation.TileRole import TileRole
from scripts.creation.BFS import BFS
from scripts.creation.PathNotFoundException import PathNotFoundException

A_WIDTH = 2
A_HEIGHT = 3
SOME_MATRIX_COORDINATES = (4, 5)
SOME_OTHER_MATRIX_COORDINATES = (6, 7)
SOME_PIXEL_COORDINATES = (8, 9)
SOME_OTHER_PIXEL_COORDINATES = (10, 11)

BFS = BFS()


def test_given_unconnected_graph_without_end_role_then_raise_path_does_not_exist():
    start_node = given_unconnected_graph_without_end_role()

    with pytest.raises(PathNotFoundException):
        BFS.find_path(start_node)


def test_given_connected_graph_with_end_role_then_path_is_valid():
    start_node = given_connected_graph_with_end_role()

    path = BFS.find_path(start_node)

    assert path


def test_given_connected_graph_without_end_role_then_raise_path_does_not_exist():
    start_node = given_connected_graph_without_end_role()

    with pytest.raises(PathNotFoundException):
        BFS.find_path(start_node)


def given_unconnected_graph_without_end_role():
    start_node = Node(SOME_MATRIX_COORDINATES, SOME_PIXEL_COORDINATES, A_WIDTH, A_HEIGHT)
    return start_node


def given_connected_graph_with_end_role():
    start_node = Node(SOME_MATRIX_COORDINATES, SOME_PIXEL_COORDINATES, A_WIDTH, A_HEIGHT)
    end_node = Node(SOME_OTHER_MATRIX_COORDINATES, SOME_OTHER_PIXEL_COORDINATES, A_WIDTH, A_HEIGHT)
    start_node.neighbors.append(end_node)
    end_node.role = TileRole.END
    return start_node


def given_connected_graph_without_end_role():
    start_node = Node(SOME_MATRIX_COORDINATES, SOME_PIXEL_COORDINATES, A_WIDTH, A_HEIGHT)
    end_node = Node(SOME_OTHER_MATRIX_COORDINATES, SOME_OTHER_PIXEL_COORDINATES, A_WIDTH, A_HEIGHT)
    start_node.neighbors.append(end_node)
    return start_node
