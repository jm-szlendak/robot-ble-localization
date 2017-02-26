from threading import Thread
from bluepy.btle import BTLEException
import rospy

class ScanWorker:
    """Wrapper of BluePy Scanner class to run in separate thread"""

    def __init__(self, scanner, scan_time=1):
        self.__scanner = scanner
        self.__scan_time = scan_time
        self.__thread = Thread(target=self.run)
        self.__thread.setDaemon(True)
        self.__stopped = False

    def run(self):
        while not self.__stopped:

            try:
                self.__scanner.scan(self.__scan_time)
            except BTLEException as e:
                rospy.logerr('BTLEException: ' + str(e) + '\nCheck your Bluetooth device. If it is turned on, this may be problem with lack of root privileges. See '
                                                          'package readme.')
                self.__stopped = True
            except Exception as e:
                pass

    def start(self):
        self.__stopped = False
        self.__thread.start()

    def stop(self):

        self.__stopped = True
        self.__thread.join()

        # print 'stopped: ' + self.__thread.name
