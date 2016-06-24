#!/usr/bin/env python
import sys
import os.path
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from scripts.scan import BeaconScanDelegate
from bluepy.btle import Scanner
import rospy

#move to parameter server:
scan_time = 1
sleep_time = 0.5

if __name__ == "__main__":
    rospy.init_node("beacon_listener")
    rospy.loginfo("Starting Beacon Listener")

    scanDelegate = BeaconScanDelegate()
    scanner = Scanner().withDelegate(scanDelegate)

    rospy.loginfo("Scanning...")

    r = rospy.Rate(1 / (scan_time + sleep_time))
    while not rospy.is_shutdown():
        devices = scanner.start()
        scanner.process(scan_time)
        r.sleep()

    rospy.spin()