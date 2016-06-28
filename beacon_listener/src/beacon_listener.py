#!/usr/bin/env python
import sys
import os.path
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from scripts.scan import BeaconScanDelegate
from scripts.beacon_container import BeaconContainer
from scripts.scan_worker import ScanWorker
from beacon_msgs.msg import Advertising
from bluepy.btle import Scanner
import rospy

# move to parameter server:
scan_time = 1
beacon_max_age = 3
publish_rate = 1

if __name__ == "__main__":
    # init ros
    rospy.init_node("beacon_listener")
    rospy.loginfo("Starting Beacon Listener")

    # create scanner
    beacon_container = BeaconContainer(beacon_max_age)
    scan_delegate = BeaconScanDelegate(container=beacon_container)
    scanner = Scanner().withDelegate(scan_delegate)
    scan_worker = ScanWorker(scanner, scan_time=scan_time)

    # start scanning
    scan_worker.start()
    rospy.loginfo("Scanning...")

    r = rospy.Rate(1 / publish_rate)
    # pub = rospy.Publisher('beacons', Advertising(), queue_size=100)
    while not rospy.is_shutdown():
        beacon_container.clean()
        beacon_container.dump()
        r.sleep()

