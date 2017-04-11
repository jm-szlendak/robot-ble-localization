from threading import Lock
from time import sleep


class BeaconDataCollector:
    def __init__(self):
        self.__lock = Lock()
        self.__buffer = []
        self.__is_collecting = False

    def wait_for_measurements(self, measurements_count):
        self.__is_collecting = True

        while len(self.__buffer) < measurements_count:
            sleep(0.1)

        self.__is_collecting = False
        data = self.__buffer
        self.__buffer = []
        return data

    def add_measurement(self, measurement):
        if not self.__is_collecting:
            return

        with self.__lock:
            self.__buffer.append(measurement)
