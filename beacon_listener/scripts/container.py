from threading import Lock
import time


class Container:
    """Container class for storing active devices. Inactive devices may be cleaned up"""

    def __init__(self, max_age=10):
        self.__beacons = dict()
        self.__lock = Lock()
        self.__max_age = max_age

    def __iter__(self):
        with self.__lock:
            for k, v in self.__beacons.items():
                yield v

    def __len__(self):
        with self.__lock:
            return len(self.__beacons)

    def clean(self):
        """Cleans up container, deleting items older than max_age attribute"""
        with self.__lock:
            self.__beacons = {addr: dev for addr, dev in self.__beacons.items() if
                              time.time() - dev.updated_at < self.__max_age}

    def insert(self, addr, dev):
        """Insert/update of device. Thread safe"""
        with self.__lock:
            self.__beacons[addr] = dev

    def get(self, addr):
        """Returns device of given address"""
        if addr in self.__beacons:
            return self.__beacons[addr]
        else:
            return None

    def dump(self):
        """Dumps device list"""
        for k, v in self.__beacons.items():
            print k, v
