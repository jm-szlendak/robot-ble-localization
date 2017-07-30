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
from beacon_msgs.srv import GetAvailableBeacons, GetAvailableLocationTags
from bluepy.btle import Scanner
import rospy


def shutdown_hook():
    rospy.loginfo("Shutting down...")
    scan_worker.stop()


if __name__ == "__main__":
    # init ros
    rospy.init_node("beacon_listener")
    rospy.loginfo("Starting Beacon Listener")
    rospy.on_shutdown(shutdown_hook)

    SCAN_TIME = rospy.get_param('beacon_listener/scan_time')
    MAX_BEACON_AGE = rospy.get_param('beacon_listener/max_beacon_age')
    PUBLISH_RATE = rospy.get_param('beacon_listener/publish_rate')
    GROUP_ID = rospy.get_param('beacon_listener/group_id')

    # create scanner
    beacon_container = Container(MAX_BEACON_AGE)
    scan_delegate = BeaconScanDelegate(container=beacon_container)
    scanner = Scanner().withDelegate(scan_delegate)
    scan_worker = ScanWorker(scanner, scan_time=SCAN_TIME)

    # start scanning
    scan_worker.start()
    rospy.loginfo("Scanning...")

    # init filtering
    adv_filter = BeaconLocalizationFilter(GROUP_ID, MAX_BEACON_AGE)
    # ros_pub_int = rospy.Publisher('/beacon_localization/location_tag/interpolated', LocationTag, queue_size=100)
    ros_pub = rospy.Publisher('/beacon_localization/location_tag', LocationTag, queue_size=100)
    publisher = PublishingWrapper(adv_filter=adv_filter, container=beacon_container, pub=ros_pub)

    # init services
    available_beacons_svc_wrapper = AvailableBeaconsServiceWrapper(beacon_container)
    available_beacons_svc = rospy.Service('/beacon_localization/get_all_available_beacons', GetAvailableBeacons, available_beacons_svc_wrapper.handler)
    rospy.loginfo("/beacon_localization/get_all_available_beacons service started")

    available_tags_svc_wrapper = AvailableLocationTagsServiceWrapper(beacon_container, adv_filter)
    available_tags_svc = rospy.Service('/beacon_localization/get_available_location_tags', GetAvailableLocationTags, available_tags_svc_wrapper.handler)
    rospy.loginfo("/beacon_localization/get_available_location_tags' service started")

    r = rospy.Rate(PUBLISH_RATE)
    while not rospy.is_shutdown():
        beacon_container.clean()
        publisher.publish()
        r.sleep()

