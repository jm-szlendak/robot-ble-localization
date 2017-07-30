import rospy
from binascii import hexlify
import time
from beacon_msgs.srv import GetBeaconDistancesResponse
from beacon_msgs.msg import BeaconPositionAndDistance


class GetDistancesServiceWrapper:
    """This service returns list of recent beacon distances"""

    def __init__(self, data_filter):
        self.__filter = data_filter

    def handler(self, req):
        res = GetBeaconDistancesResponse()
        res.measurements = [
            BeaconPositionAndDistance(bid=k, updated_at=rospy.Time.from_sec(time.time()), distance=v.rssi)
            for k, v in self.__filter.get_all_values().items()
        ]

        return res
