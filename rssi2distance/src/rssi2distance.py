#!/usr/bin/env python
import sys
import os.path

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from beacon_msgs.msg import LocationTag, BeaconPositionAndDistance
from geometry_msgs.msg import Pose, Point
from scripts.filters import OnlyRecentValueFilter, MovingAverageFilter, ProbabilisticFilter
from scripts.services import GetDistancesServiceWrapper
from beacon_msgs.srv import GetBeaconDistances
from scripts.rssi2distance_model import Power10Model
from scripts.beacon_map import BeaconMap
import rospy
import time


recent_val_filter = OnlyRecentValueFilter()
moving_avg_filter = MovingAverageFilter()
probabilistic_filter = ProbabilisticFilter(0.25, 0.25, 1)

recent_pub = None
moving_avg_pub = None
probabilistic_pub = None

beacon_map = None


def setup_map(beacons):
    map = BeaconMap()
    for beacon in beacons:
        model = Power10Model(*beacon['modelParams'])
        x = beacon['x']
        y = beacon['y']
        z = beacon['z']
        bid = beacon['id']
        name = beacon['name']
        map.add_beacon(bid, x, y, z, model, name)

    return map


def publish_distance(bid, filter, publisher):
    item = filter.get_value(bid)

    publisher.publish(BeaconPositionAndDistance(
        bid=bid,
        updated_at=rospy.Time.from_sec(time.time()),
        distance=beacon_map.get(bid).model.apply(item.rssi),
        rssi=item.rssi,
        name=beacon_map.get(bid).name,
        pose=Pose(position=Point(
            x=beacon_map.get(bid).x,
            y=beacon_map.get(bid).y,
            z=beacon_map.get(bid).z
        ))
    ))


def callback(data):
    recent_val_filter.put(data.bid, data)
    moving_avg_filter.put(data.bid, data)
    probabilistic_filter.put(data.bid, data)

    publish_distance(data.bid, recent_val_filter, recent_pub)
    publish_distance(data.bid, moving_avg_filter, moving_avg_pub)
    publish_distance(data.bid, probabilistic_filter, probabilistic_pub)


if __name__ == "__main__":
    # init ros
    rospy.init_node("rssi2distance")
    rospy.loginfo("Starting RSSI2Distance node")

    rospy.Subscriber('beacon_localization/location_tag', LocationTag, callback)

    beacons = rospy.get_param('/beacon_localization/map/beacons')
    beacon_map = setup_map(beacons)

    recent_val_srv_wrapper = GetDistancesServiceWrapper(recent_val_filter, beacon_map)
    moving_avg_srv_wrapper = GetDistancesServiceWrapper(moving_avg_filter, beacon_map)
    probabilistic_srv_wrapper = GetDistancesServiceWrapper(probabilistic_filter, beacon_map)

    recent_val_srv = rospy.Service('/beacon_localization/distances/recent', GetBeaconDistances,
                                   recent_val_srv_wrapper.handler)
    moving_avg_srv = rospy.Service('/beacon_localization/distances/moving_average', GetBeaconDistances,
                                   moving_avg_srv_wrapper.handler)
    probabilistic_srv = rospy.Service('/beacon_localization/distances/probabilistic', GetBeaconDistances,
                                      probabilistic_srv_wrapper.handler)

    recent_pub = rospy.Publisher('/beacon_localization/distances/recent', BeaconPositionAndDistance, queue_size=100)
    moving_avg_pub = rospy.Publisher('/beacon_localization/distances/moving_average', BeaconPositionAndDistance, queue_size=100)
    probabilistic_pub = rospy.Publisher('/beacon_localization/distances/probabilistic', BeaconPositionAndDistance, queue_size=100)
    rospy.spin()
