#!/usr/bin/env python
import sys
import os.path
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from beacon_msgs.msg import LocationTag
from scripts.filters import OnlyRecentValueFilter, MovingAverageFilter
from scripts.services import GetDistancesServiceWrapper
from beacon_msgs.srv import GetBeaconDistances
from scripts.rssi2distance_model import Power10Model
from scripts.beacon_map import BeaconMap
import rospy


recent_val_filter = OnlyRecentValueFilter()
moving_avg_filter = MovingAverageFilter()

def setup_map(beacons):
    map = BeaconMap()
    for beacon in beacons:
        model = Power10Model(*beacon['modelParams'])
        x = beacon['x']
        y = beacon['y']
        z = beacon['z']
        id = beacon['id']
        map.add_beacon(id, x, y, z, model)

    return map


def callback(data):
    recent_val_filter.put(data.bid, data)
    moving_avg_filter.put(data.bid, data)

if __name__ == "__main__":
    # init ros
    rospy.init_node("rssi2distance")
    rospy.loginfo("Starting RSSI2Distance node")

    rospy.Subscriber('beacon_localization/location_tag', LocationTag, callback)

    beacons = rospy.get_param('/beacon_localization/map/beacons')

    beacon_map = setup_map(beacons)

    recent_val_srv_wrapper = GetDistancesServiceWrapper(recent_val_filter, beacon_map)
    moving_avg_srv_wrapper = GetDistancesServiceWrapper(moving_avg_filter, beacon_map)
    recent_val_srv = rospy.Service('/beacon_localization/distances/recent', GetBeaconDistances,
                                   recent_val_srv_wrapper.handler)
    moving_avg_srv = rospy.Service('/beacon_localization/distances/moving_average', GetBeaconDistances,
                                   moving_avg_srv_wrapper.handler)
    rospy.spin()

