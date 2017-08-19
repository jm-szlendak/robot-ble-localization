class Power10Model(object):

    def __init__(self, *args):
        self.A = args[0]
        self.B = args[1]

    def apply(self, x):
        """Applies model to given rssi, returning distance"""
        value = 10 ** ((-x - self.A)/self.B)  # self.A * np.exp(-self.B * x) + self.C
        return value
