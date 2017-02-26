#!/usr/bin/env python
import sys
import os.path
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from scripts.scan import BeaconScanDelegate
from scripts.container import Container
from scripts.scan_worker import ScanWorker
from scripts.publisher import PublishingWrapper
from scripts.beacon_localization import BeaconLocalizationFilter
from scripts.services import *
from beacon_msgs.msg import LocationTag
from bluepy.btle import Scanner
import rospy

# move to parameter server:
scan_time = 1
beacon_max_age = 3
publish_rate = 1
group_id = 0xAAAA
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
    beacon_container = Container(beacon_max_age)
    scan_delegate = BeaconScanDelegate(container=beacon_container)
    scanner = Scanner().withDelegate(scan_delegate)
    scan_worker = ScanWorker(scanner, scan_time=scan_time)

    # start scanning
    scan_worker.start()
    rospy.loginfo("Scanning...")

    # init filtering
    adv_filter = BeaconLocalizationFilter(group_id, beacon_max_age)
    ros_pub = rospy.Publisher('/beacon_localization/location_tag', LocationTag, queue_size=100)
    publisher = PublishingWrapper(adv_filter=adv_filter, container=beacon_container, pub=ros_pub)

    # init services
    available_devs_svc_wrapper = AvaliableDevicesServiceWrapper(beacon_container)
    available_devs_svc = rospy.Service('/beacon_localization/get_available_devices', GetAvailableDevices, available_devs_svc_wrapper.handler)
    rospy.loginfo("'/beacon_localization/get_available_devices' service started")

    r = rospy.Rate(1 / publish_rate)
    while not rospy.is_shutdown():
        beacon_container.clean()
        publisher.publish()
        r.sleep()
