#!/usr/bin/env python
import sys
import os.path
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from beacon_msgs.msg import LocationTag
from scripts.beacon_buffer import BeaconBuffer

import rospy

MAX_BUFFERED_BEACON_AGE = 5
buffer = BeaconBuffer(MAX_BUFFERED_BEACON_AGE)

def callback(data):
    buffer.put(data)


if __name__ == "__main__":
    # init ros
    rospy.init_node("rssi2distance")
    rospy.loginfo("Starting RSSI2Distance node")

    rospy.Subscriber('beacon_localization/location_tag', LocationTag, callback)

    rospy.spin()

