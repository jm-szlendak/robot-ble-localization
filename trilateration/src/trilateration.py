#!/usr/bin/env python
import sys
import os.path
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import rospy
from beacon_msgs.srv import GetBeaconDistances
from scripts.basic_trilateration import BasicTrilateration

def get_param(name, default_val):
    try:
        return rospy.get_param('beacon_listener/trilateration/distance_filtering')
    except KeyError:
        return default_val

if __name__ == "__main__":
    # init ros
    rospy.init_node('trilateration')
    rospy.loginfo("Starting Trilateration node")

    TAG_DISTANCE_FILTERING = get_param('beacon_listener/trilateration/distance_filtering', 'recent')
    LOCALIZATION_RATE = get_param('beacon_listener/trilateration/localization_rate', 1)

    service_path = '/beacon_localization/distances/' + TAG_DISTANCE_FILTERING
    rospy.wait_for_service(service_path)
    get_beacons_srv = rospy.ServiceProxy(service_path, GetBeaconDistances)

    r = rospy.Rate(LOCALIZATION_RATE)
    while not rospy.is_shutdown():
        beacons_with_distances = get_beacons_srv.call()

        trilateration_engine.ca(beacons_with_distances)
        r.sleep()

