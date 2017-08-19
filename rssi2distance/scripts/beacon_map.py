class BeaconMap(object):
    def __init__(self):
        self.beacons = dict()
        pass

    def add_beacon(self, bid, x_pos, y_pos, z_pos, model):
        self.beacons[bid] = Beacon(bid, x_pos, y_pos, z_pos, model)

        return self

    def get(self, bid):
        if bid in self.beacons.keys():
            return self.beacons[bid]
        else:
            return None


class Beacon(object):
    def __init__(self, bid, x, y, z, model):
        self.bid = bid
        self.x = x
        self.y = y
        self.z = z
        self.model = model
