import rospy
from binascii import hexlify

from beacon_msgs.srv import GetAvailableDevices, GetAvailableDevicesResponse
from beacon_msgs.msg import Advertising


class AvaliableDevicesServiceWrapper:
    """Encapsulates avaliable_devices service
        This service returns list of devices available to connect with"""

    def __init__(self, container):
        self.__container = container

    def handler(self, req):
        req = GetAvailableDevicesResponse()
        rospy.logdebug("Called avaliable_devices service")

        for beacon in self.__container:
            req.beacons.append(self.beacon_to_ros_msg(beacon))

        return req

    @staticmethod
    def beacon_to_ros_msg(beacon):
        return Advertising(addr=beacon.addr.encode('ascii'),
                           updated_at=rospy.Time.from_sec(beacon.updated_at),
                           rssi=beacon.rssi,
                           data=hexlify(beacon.raw))

