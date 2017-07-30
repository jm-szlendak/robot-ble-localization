import math
import numpy as np

class ExponentialModel(object):

    def __init__(self, *args):
        self.A = args[0]
        self.B = args[1]
        self.C = args[2]

    def apply(self, rssi):
        """Applies model to given rssi, returning distance"""
        return self.A * np.exp(-self.B * float(rssi)) + self.C
