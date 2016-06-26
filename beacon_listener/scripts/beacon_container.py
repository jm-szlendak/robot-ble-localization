from threading import Lock
import time


class BeaconContainer:
    """Container class for storing active devices. Inactive devices may be cleaned up"""

    def __init__(self, max_age=10):
        self.__beacons = dict()
        self.__lock = Lock()
        self.__max_age = max_age

    def clean(self):
        with self.__lock:
            self.__beacons = {addr: dev for addr, dev in self.__beacons.items() if
                              time.time() - dev.updated_at < self.__max_age}

    def insert(self, addr, dev):
        with self.__lock:
            self.__beacons[addr] = dev

    def get(self, addr):
        if addr in self.__beacons:
            return self.__beacons[addr]
        else:
            return None
