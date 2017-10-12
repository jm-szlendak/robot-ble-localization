import numpy as np
from scipy import optimize
import utils


class IterativeTrilaterationEngine(object):
    def __init__(self, error_fn='mse', initial_guess='beacon_center'):
        if error_fn is 'mse':
            self.error_fn = utils.mse

        if initial_guess is 'beacon_center':
            self.initial_guess_fn = utils.initial_guess_center_of_mass

    def calculate(self, beacons):
        positions = [np.array([beacon.pose.position.x, beacon.pose.position.y]) for beacon in beacons]
        distances = [beacon.distance for beacon in beacons]

        estimate = self.initial_guess_fn(positions)
        error = float('inf')
        delta_len = float('inf')
        safe_ctr = 1000
        while error > 0.001 and safe_ctr > 0 and delta_len > 1e-6:
            delta = self.calculate_delta(estimate, positions, distances)
            delta_len = np.linalg.norm(delta)
            print estimate, delta
            estimate += delta
            safe_ctr -= 1


        return [estimate[0], estimate[1], 0]

    def calculate_delta(self, initial, positions, distances):

        B = np.matrix([
            [
                (pos[0]-initial[0])/(np.linalg.norm(pos-initial)),
                (pos[1] - initial[1]) / (np.linalg.norm(pos - initial)),
            ]
            for pos in positions
        ])

        f = -1 * np.matrix([
            [
                distances[i] - (np.linalg.norm(positions[i]-initial))
            ] for i in range(0, len(distances))
        ])

        return np.array(np.linalg.inv(B.transpose()*B)*B.transpose()*f).flatten()


