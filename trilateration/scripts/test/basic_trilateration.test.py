import unittest

from beacon_msgs.msg import BeaconPositionAndDistance
from geometry_msgs.msg import Pose, Point
from basic_trilateration import BasicTrilaterationEngine


class UtilsTestCase(unittest.TestCase):

    def test_basic_trilateration(self):
        t = BasicTrilaterationEngine()

        beacons = [
            BeaconPositionAndDistance(pose=Pose(position=Point(x=1, y=0)), distance=1),
            BeaconPositionAndDistance(pose=Pose(position=Point(x=0, y=1)), distance=1),
            BeaconPositionAndDistance(pose=Pose(position=Point(x=-1, y=0)), distance=1),
        ]

        result = t.calculate(beacons)

        self.assertEqual(result[0], 0)
        self.assertEqual(result[1], 0)

