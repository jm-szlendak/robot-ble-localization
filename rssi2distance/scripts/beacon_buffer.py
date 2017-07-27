import time

class BeaconBuffer:
    def __init__(self, max_age):
        self.__buffer = dict()
        self.__max_age = max_age

    def put(self, beacon):
        if beacon.bid in self.__buffer:
            self.___buffer[beacon.bid] = [beacon]
        else:
            self.__buffer[beacon.bid].append(beacon)

        # pretty naive trick to prevent extensive grow of buffer: leave at most 20 recent items in buffer
        self.__buffer[beacon.bid][-20:]

    def get_newest_beacons(self):
        self.cleanup()
        beacons = []
        for beacon_buffer in self.__buffer.values():
            if len(beacon_buffer) > 0:
                beacons.append(beacon_buffer[-1])
        return beacons

    def cleanup(self):
        for beacon_id in self.__buffer.keys():
            self.__buffer[beacon_id] = \
                [beacon for beacon in self.__buffer[beacon_id] if time.time() - beacon.updated_at < self.__max_age]