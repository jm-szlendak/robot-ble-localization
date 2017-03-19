import rospy
from binascii import hexlify

from beacon_msgs.srv import GetAvailableBeaconsResponse, GetAvailableLocationTagsResponse
from beacon_msgs.msg import Advertising, LocationTag


class AvailableBeaconsServiceWrapper:
    """Encapsulates available_devices service
        This service returns list of devices available to connect with"""

    def __init__(self, container):
        self.__container = container

    def handler(self, req):
        req = GetAvailableBeaconsResponse()
        rospy.loginfo("Called available_devices service")

        for beacon in self.__container:
            req.beacons.append(self.beacon_to_ros_msg(beacon))

        return req

    @staticmethod
    def beacon_to_ros_msg(beacon):
        return Advertising(addr=beacon.addr.encode('ascii'),
                           updated_at=rospy.Time.from_sec(beacon.updated_at),
                           rssi=beacon.rssi,
                           data=hexlify(beacon.raw))


class AvailableLocationTagsServiceWrapper:
    """Encapsulates available_devices service
        This service returns list of devices available to connect with"""

    def __init__(self, container, filter):
        self.__container = container
        self.__filter = filter

    def handler(self, req):
        req = GetAvailableLocationTagsResponse()
        rospy.loginfo("Called available_devices service")

        filtered_list = self.__filter.do_filter(self.__container)

        for beacon in filtered_list:
            req.tags.append(beacon)

        return req


