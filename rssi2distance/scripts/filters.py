import time
from threading import Lock
from beacon_msgs.msg import LocationTag


class AbstractFilter(object):

    def __init__(self, max_age=5):
        self.values = dict()
        self.buffers = dict()
        self.__max_age = max_age
        self.lock = Lock()

    def get_value(self, bid):
        with self.lock:
            return self.values[bid]

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


class ProbabilisticFilter(AbstractFilter):

    def __init__(self,  a, b, Ts, max_age = 5):
        super(ProbabilisticFilter, self).__init__(max_age)
        self.__a = a
        self.__b = b
        self.__Ts = Ts
        self.__R_est = dict()
        self.__R_pred = dict()
        self.__V_est  = dict()
        self.__V_pred = dict()

    def put(self, bid, item):
        with self.lock:
            if bid not in self.buffers.keys():
                self.buffers[bid] = []
                self.values[bid] = None
                self.__R_est[bid] = float(item.rssi)
                self.__R_pred[bid] = float(item.rssi)
                self.__V_est[bid] = 0.0
                self.__V_pred[bid] = 0.0

        # Estimation
        self.__R_est[bid] = self.__R_pred[bid] + self.__a * (float(item.rssi) - self.__R_pred[bid])
        self.__V_est[bid] = self.__V_pred[bid] + (self.__b / self.__Ts) * (float(item.rssi) - self.__R_pred[bid])

        # Prediction
        self.__R_pred[bid] = self.__R_est[bid] + self.__V_est[bid] * self.__Ts
        self.__V_pred[bid] = self.__V_est[bid]

        self.values[bid] = LocationTag(gid=item.gid, bid=bid, stamp=time.time(), rssi=self.__R_est[bid])