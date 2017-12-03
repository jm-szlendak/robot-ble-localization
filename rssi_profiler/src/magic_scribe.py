#!/usr/bin/env python
import sys
import os.path

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from beacon_msgs.srv import GetBeaconDistances
import numpy as np
import rospy
import tf
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
    args = argparser.parse_args(rospy.myargv()[1:])

    rospy.loginfo("Starting !!!MAGICAL!!! RSSI Scribe")
    rospy.loginfo(
        "If encountering problems with transform, make sure  base link and beacons TF are published and sychronized \
         (refer to https://answers.ros.org/question/123256/sync-ros-bag-timestamps-with-ros-system/ if using rosbag")

    base_frame = args.base
    filter_type = args.filter
    result = {}

    beacons = rospy.get_param('/batman/beacon_localization/map/beacons')

    rospy.sleep(0.1)
    t = tf.TransformListener()
    for beacon in beacons:
        frame = '/beacon_' + beacon['name'] + '_' + beacon['id']
        t.waitForTransform(frame, base_frame, rospy.Time(0), rospy.Duration(10.0))
        (trans, r) = t.lookupTransform(frame, base_frame, rospy.Time(0))

        result[beacon['id']] = np.linalg.norm(trans)

    rospy.wait_for_service('/batman/beacon_localization/distances/'+filter_type)
    distances_srv = rospy.ServiceProxy('/batman/beacon_localization/distances/'+filter_type, GetBeaconDistances)
    distances_response = distances_srv.call()

    for id, dist in result.items():
        res = [response for response in distances_response.measurements if response.bid == id][0]
        try:
            print 'beacon_' + res.name + '_' + res.bid + '.txt'
            with open('beacon_' + res.name + '_' + res.bid + '.txt', 'a') as outfile:

                line = str(dist)
                line += '\t'
                line += str(res.rssi)
                line += '\n'
                outfile.write(line)
        except IOError as e:
            print 'Unable to open file: ', e
            exit(1)


if __name__ == "__main__":
    main()
