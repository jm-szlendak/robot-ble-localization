import time

from threading import Lock

from beacon_msgs.msg import LocationTag

class AbstractFilter(object):

    def __init__(self, max_age=5):
        self.values = dict()
        self.buffers = dict()
        self.__max_age = max_age
        self.lock = Lock()

    def get_all_values(self):
        with self.lock:
            return self.values

    def cleanup(self):
        for beacon_id in self.values.keys():
            self.buffers[beacon_id] = \
                [beacon for beacon in self.buffers[beacon_id] if time.time() - beacon.updated_at < self.__max_age]


class OnlyRecentValueFilter(AbstractFilter):
    """A filter which returns only most recent value"""
    def __init__(self, max_age=5):
        super(OnlyRecentValueFilter, self).__init__(max_age)

    def put(self, bid, item):
        with self.lock:
            self.values[bid] = item


class MovingAverageFilter(AbstractFilter):
    """A filter which returns moving average of last values"""

    def __init__(self, max_age=5, window_length=5):
        super(MovingAverageFilter, self).__init__(max_age)
        self.__window_len = window_length

    def put(self, bid, item):
        with self.lock:
            if bid not in self.buffers.keys():
                print 'will create buffer for bid ' + bid
                self.buffers[bid] = []
                self.values[bid] = None

            self.buffers[bid].append(item)
            # cut buffer to be not longer than window
            self.buffers[bid] = self.buffers[bid][-1*self.__window_len:]

            # recalculate average
            average = LocationTag(gid=self.buffers[bid][0].gid, bid=bid, stamp=time.time(), rssi=0)
            for item in self.buffers[bid]:
                average.rssi = average.rssi + item.rssi

            average.rssi /= float(len(self.buffers[bid]))

            self.values[bid] = average
