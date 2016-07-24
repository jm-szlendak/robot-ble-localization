from threading import Thread


class ScanWorker:
    """Wrapper of BluePy Scanner class to run in separate thread"""

    def __init__(self, scanner, scan_time=1):
        self.__scanner = scanner
        self.__scan_time = scan_time
        self.__thread = Thread(target=self.run)
        self.__stopped = False

    def run(self):
        while not self.__stopped:
            self.__scanner.start()
            self.__scanner.process(self.__scan_time)

    def start(self):
        self.__stopped = False
        self.__thread.start()

    def stop(self):
        self.__stopped = True
