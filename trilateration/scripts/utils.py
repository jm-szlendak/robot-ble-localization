import numpy as np

def mse(x, positions, distances):
    """Calculates Mean Square Error trilateration distance and measured distance

        x -- position calculated with trilateration (numpy array)
        positions -- beacon positions on map (list of numpy arrays)
        distances -- measured distances to beacons (list of numbers)
    """

    MSE = 0.0
    for pos, dist in zip(positions, distances):
        d = np.linalg.norm(x - pos)
        MSE += (d - dist) ** 2

    return MSE / len(distances)


def initial_guess_center_of_mass(points):
    return reduce(lambda x, y: x + y, points, np.array([0, 0])) / len(points)