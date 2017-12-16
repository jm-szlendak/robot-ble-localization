#!/usr/bin/env python
import sys
import os.path

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from beacon_msgs.srv import GetBeaconDistances
import numpy as np
import rospy
import tf
import yaml
from argparse import ArgumentParser


def main():
    # init ros
    rospy.init_node("rssi_scribe")

    argparser = ArgumentParser()
    argparser.add_argument('--base',
                           help='BLE sensor base frame, default: /batman/base_link',
                           default='/batman/base_link')
    argparser.add_argument('--filter',
                           help='RSSI filter to be used: probabilistic(default), moving_average, recent',
                           default='probabilistic')

    argparser.add_argument('--out',
                           help='output file name')

    args = argparser.parse_args(rospy.myargv()[1:])

    rospy.loginfo("Starting !!!MAGICAL!!! RSSI Scribe")
    rospy.loginfo(
        "If encountering problems with transform, make sure  base link and beacons TF are published and sychronized \
         (refer to https://answers.ros.org/question/123256/sync-ros-bag-timestamps-with-ros-system/ if using rosbag")

    base_frame = args.base
    filter_type = args.filter
    result = {}


    rospy.sleep(0.1)
    t = tf.TransformListener()

    t.waitForTransform('map', base_frame, rospy.Time(0), rospy.Duration(10.0))
    (trans, r) = t.lookupTransform('map', base_frame, rospy.Time(0))

    result['x'] = trans[0]
    result['y'] = trans[1]
    result['rssi'] = {}

    rospy.wait_for_service('/batman/beacon_localization/distances/'+filter_type)
    distances_srv = rospy.ServiceProxy('/batman/beacon_localization/distances/'+filter_type, GetBeaconDistances)
    distances_response = distances_srv.call()

    for measurement in distances_response.measurements:
        result['rssi'][measurement.name] = measurement.rssi

    with open(args.out, 'a') as outfile:
        outfile.write(yaml.dump(result) + '---\n')

    print 'done'



if __name__ == "__main__":
    main()
