import unittest
import numpy as np

import utils


class UtilsTestCase(unittest.TestCase):
    def test_mse_gives_0(self):

        x = np.array([0, 0])
        positions = [np.array([1, 0])]
        distances = [1]
        self.assertEqual(utils.mse(x, positions, distances), 0)

    def test_mse_gives_non_zero(self):

        x = np.array([0, 0])
        positions = [np.array([1, 0])]
        distances = [1.5]
        self.assertEqual(utils.mse(x, positions, distances), 0.25)

    def test_mse_gives_non_zero_for_multiple_beacons(self):
        x = np.array([0, 0])
        positions = [np.array([1, 0]), np.array([1, 0])]
        distances = [1.5, 2]
        self.assertEqual(utils.mse(x, positions, distances), 1.25 / 2)

    def test_initial_guess_center_of_mass(self):
        beacons = [np.array([1, 0]), np.array([0, 1]), np.array([0, -1]), np.array([-1, 0])]

        center = utils.initial_guess_center_of_mass(beacons)

        self.assertEqual(center[0], 0)
        self.assertEqual(center[1], 0)
