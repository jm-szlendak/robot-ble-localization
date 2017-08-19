import numpy as np
from scipy import optimize
import utils


class BasicTrilaterationEngine(object):
    def __init__(self, error_fn='mse', initial_guess='beacon_center'):
        if error_fn is 'mse':
            self.error_fn = utils.mse

        if initial_guess is 'beacon_center':
            self.initial_guess_fn = utils.initial_guess_center_of_mass

    def calculate(self, beacons):
        positions = [np.array([beacon.pose.position.x, beacon.pose.position.y]) for beacon in beacons]
        distances = [beacon.distance for beacon in beacons]

        result = optimize.minimize(
            self.error_fn,
            self.initial_guess_fn(positions),
            args=(positions, distances),
            method='L-BFGS-B',
            options={
                'ftol': 1e-5,
                'maxiter': 1e+7
            })

        return [result.x[0], result.x[1], 0]
