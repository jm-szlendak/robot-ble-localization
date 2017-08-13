import unittest
from basic_trilateration import BasicTrilateration
from beacon_msgs.msg import BeaconPositionAndDistance


class UtilsTestCase(unittest.TestCase):

    def test_basic_trilateration(self):
        t = BasicTrilateration()

        beacons = [
            BeaconPositionAndDistance(x=1, y=0, distance=1),
            BeaconPositionAndDistance(x=0, y=1, distance=1),
            BeaconPositionAndDistance(x=-1, y=0, distance=1),
        ]

        result = t.calculate(beacons)

        self.assertEqual(result[0], 0)
        self.assertEqual(result[1], 0)

