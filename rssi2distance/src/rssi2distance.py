#!/usr/bin/env python
import sys
import os.path
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from beacon_msgs.msg import LocationTag
from scripts.filters import OnlyRecentValueFilter, MovingAverageFilter
from scripts.services import GetDistancesServiceWrapper
from beacon_msgs.srv import GetBeaconDistances

import rospy


recent_val_filter = OnlyRecentValueFilter()
moving_avg_filter = MovingAverageFilter()

def callback(data):
    recent_val_filter.put(data.bid, data)
    moving_avg_filter.put(data.bid, data)

if __name__ == "__main__":
    # init ros
    rospy.init_node("rssi2distance")
    rospy.loginfo("Starting RSSI2Distance node")

    rospy.Subscriber('beacon_localization/location_tag', LocationTag, callback)

    recent_val_srv_wrapper = GetDistancesServiceWrapper(recent_val_filter)
    moving_avg_srv_wrapper = GetDistancesServiceWrapper(moving_avg_filter)
    recent_val_srv = rospy.Service('/beacon_localization/distances/recent', GetBeaconDistances,
                                   recent_val_srv_wrapper.handler)
    moving_avg_srv = rospy.Service('/beacon_localization/distances/moving_average', GetBeaconDistances,
                                   moving_avg_srv_wrapper.handler)
    rospy.spin()

