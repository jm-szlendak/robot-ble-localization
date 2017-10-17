#!/usr/bin/env python
import sys
import os.path

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from beacon_msgs.msg import LocationTag, BeaconsScan, BeaconPositionAndDistance
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Header
from scripts.filters import OnlyRecentValueFilter, MovingAverageFilter, ProbabilisticFilter
from scripts.services import GetDistancesServiceWrapper
from beacon_msgs.srv import GetBeaconDistances
from scripts.rssi2distance_model import Power10Model
from scripts.beacon_map import BeaconMap
import rospy
import time


recent_val_filter = OnlyRecentValueFilter()
moving_avg_filter = MovingAverageFilter()
probabilistic_filter = ProbabilisticFilter(0.35, 0.02, 1)

recent_pub = None
moving_avg_pub = None
probabilistic_pub = None

beacon_map = None


radio_frame = 'radio_base_link'

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
    hdr = Header(stamp=rospy.Time.now(), frame_id=radio_frame)
    msg = BeaconsScan(
        header=hdr,
        beacons=[
            BeaconPositionAndDistance(
                header=hdr,
                bid=k,
                updated_at=rospy.Time.from_sec(time.time()),
                distance=beacon_map.get(k).model.apply(v.rssi),
                rssi=v.rssi,
                name=beacon_map.get(k).name,
                pose=Pose(position=Point(
                    x=beacon_map.get(k).x,
                    y=beacon_map.get(k).y,
                    z=beacon_map.get(k).z
                ))
            ) for k, v in filter.get_all_values().items()
        ]
    )
    publisher.publish(msg)


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

    radio_frame = rospy.get_namespace()[1:] + radio_frame
    rospy.Subscriber('beacon_localization/location_tag', LocationTag, callback)

    beacons = rospy.get_param('beacon_localization/map/beacons')
    beacon_map = setup_map(beacons)

    recent_val_srv_wrapper = GetDistancesServiceWrapper(recent_val_filter, beacon_map)
    moving_avg_srv_wrapper = GetDistancesServiceWrapper(moving_avg_filter, beacon_map)
    probabilistic_srv_wrapper = GetDistancesServiceWrapper(probabilistic_filter, beacon_map)

    recent_val_srv = rospy.Service('beacon_localization/distances/recent', GetBeaconDistances,
                                   recent_val_srv_wrapper.handler)
    moving_avg_srv = rospy.Service('beacon_localization/distances/moving_average', GetBeaconDistances,
                                   moving_avg_srv_wrapper.handler)
    probabilistic_srv = rospy.Service('beacon_localization/distances/probabilistic', GetBeaconDistances,
                                      probabilistic_srv_wrapper.handler)

    recent_pub = rospy.Publisher('beacon_localization/distances/recent', BeaconsScan, queue_size=100)
    moving_avg_pub = rospy.Publisher('beacon_localization/distances/moving_average', BeaconsScan, queue_size=100)
    probabilistic_pub = rospy.Publisher('beacon_localization/distances/probabilistic', BeaconsScan, queue_size=100)
    rospy.spin()
