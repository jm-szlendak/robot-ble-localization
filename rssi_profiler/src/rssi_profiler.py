#!/usr/bin/env python
import sys
import os.path
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from beacon_msgs.srv import RSSIProfile
from beacon_msgs.msg import LocationTag

from scripts.services import RSSIProfileServiceWrapper
from scripts.beacon_data_collector import BeaconDataCollector

import rospy

bridge = BeaconDataCollector()


def callback(data):
    bridge.add_measurement(data)

if __name__ == "__main__":
    # init ros
    rospy.init_node("rssi_profile")
    rospy.loginfo("Starting RSSI Profiler")

    rospy.Subscriber('/batman/beacon_localization/location_tag', LocationTag, callback)

    # init services
    rssi_profiler_srv_wrapper = RSSIProfileServiceWrapper(bridge, 10)
    rssi_profiler_srv = rospy.Service('/batman/beacon_localization/rssi_profile', RSSIProfile, rssi_profiler_srv_wrapper.handler)
    rospy.loginfo("/beacon_localization/get_all_available_beacons service started")

    rospy.spin()

