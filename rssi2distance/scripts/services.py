import rospy
import time
from beacon_msgs.srv import GetBeaconDistancesResponse
from beacon_msgs.msg import BeaconPositionAndDistance
from geometry_msgs.msg import Pose, Point


class GetDistancesServiceWrapper:
    """This service returns list of recent beacon distances"""

    def __init__(self, data_filter, beacon_map):
        self.__filter = data_filter
        self.__beacon_map = beacon_map

    def handler(self, req):
        res = GetBeaconDistancesResponse()
        res.measurements = [
            BeaconPositionAndDistance(
                bid=k,
                updated_at=rospy.Time.from_sec(time.time()),
                distance=self.__beacon_map.get(k).model.apply(v.rssi),
                rssi=v.rssi,
                name=self.__beacon_map.get(k).name,
                pose=Pose(position=Point(
                    x=self.__beacon_map.get(k).x,
                    y=self.__beacon_map.get(k).y,
                    z=self.__beacon_map.get(k).z
                ))
            ) for k, v in self.__filter.get_all_values().items()
        ]

        return res
