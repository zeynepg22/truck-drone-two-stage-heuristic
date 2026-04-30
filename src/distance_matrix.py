import numpy as np


def compute_distance_matrix(nodes_df):
    """
    Computes Euclidean distance matrix between all nodes.
    """
    coordinates = nodes_df[["x", "y"]].values
    n = len(coordinates)

    distance_matrix = np.zeros((n, n))

    for i in range(n):
        for j in range(n):
            distance_matrix[i][j] = np.linalg.norm(coordinates[i] - coordinates[j])

    return distance_matrix


def compute_time_matrix(distance_matrix, speed):
    """
    Converts distance matrix into travel time matrix.
    """
    return distance_matrix / speed