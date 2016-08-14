#!/usr/bin/env python
import sys
import os.path
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from scripts.scan import BeaconScanDelegate
from scripts.beacon_container import BeaconContainer
from scripts.scan_worker import ScanWorker
from scripts.beacon_publisher import BeaconPublishingWrapper
from scripts.beacon_localization import BeaconLocalizationFilter
from beacon_msgs.msg import LocationTag
from bluepy.btle import Scanner
import rospy

# move to parameter server:
scan_time = 1
beacon_max_age = 3
publish_rate = 1
group_id = 0x3412
manufacturer_id = 0xffff


def shutdown_hook():
    rospy.loginfo("Shutting down...")
    scan_worker.stop()


if __name__ == "__main__":
    # init ros
    rospy.init_node("beacon_listener")
    rospy.loginfo("Starting Beacon Listener")
    rospy.on_shutdown(shutdown_hook)
    # create scanner
    beacon_container = BeaconContainer(beacon_max_age)
    scan_delegate = BeaconScanDelegate(container=beacon_container)
    scanner = Scanner().withDelegate(scan_delegate)
    scan_worker = ScanWorker(scanner, scan_time=scan_time)

    # start scanning
    scan_worker.start()
    rospy.loginfo("Scanning...")

    r = rospy.Rate(1 / publish_rate)
    adv_filter = BeaconLocalizationFilter(group_id, beacon_max_age)
    ros_pub = rospy.Publisher('/beacon_localization/location_tag', LocationTag, queue_size=100)
    publisher = BeaconPublishingWrapper(adv_filter=adv_filter, container=beacon_container, pub=ros_pub)

    while not rospy.is_shutdown():
        beacon_container.clean()
        publisher.publish()
        r.sleep()
