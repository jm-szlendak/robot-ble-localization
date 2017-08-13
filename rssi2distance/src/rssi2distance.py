#!/usr/bin/env python
import sys
import os.path
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from beacon_msgs.msg import LocationTag
from scripts.filters import OnlyRecentValueFilter, MovingAverageFilter
from scripts.services import GetDistancesServiceWrapper
from beacon_msgs.srv import GetBeaconDistances
from scripts.rssi2distance_model import ExponentialModel
from scripts.beacon_map import BeaconMap
import rospy


recent_val_filter = OnlyRecentValueFilter()
moving_avg_filter = MovingAverageFilter()
model = ExponentialModel(8.53239738e-33,   1.00000000e+00,   3.06811272e+00) #dummy model, each beacon should have its own model!

beacon_map = BeaconMap()

beacon_map\
    .add_beacon('aa5aab45bdcf', 0.3, 0, model)\
    .add_beacon('237b44757adc', 2.8, 0, model)\
    .add_beacon('aaa25624c9e4', 2.1, 2.91, model)

def callback(data):
    recent_val_filter.put(data.bid, data)
    moving_avg_filter.put(data.bid, data)

if __name__ == "__main__":
    # init ros
    rospy.init_node("rssi2distance")
    rospy.loginfo("Starting RSSI2Distance node")

    rospy.Subscriber('beacon_localization/location_tag', LocationTag, callback)

    recent_val_srv_wrapper = GetDistancesServiceWrapper(recent_val_filter, beacon_map)
    moving_avg_srv_wrapper = GetDistancesServiceWrapper(moving_avg_filter, beacon_map)
    recent_val_srv = rospy.Service('/beacon_localization/distances/recent', GetBeaconDistances,
                                   recent_val_srv_wrapper.handler)
    moving_avg_srv = rospy.Service('/beacon_localization/distances/moving_average', GetBeaconDistances,
                                   moving_avg_srv_wrapper.handler)
    rospy.spin()

