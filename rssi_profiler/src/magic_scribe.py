#!/usr/bin/env python
import sys
import os.path

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from beacon_msgs.srv import GetBeaconDistances
import numpy as np
import rospy
import tf


def main():
    # init ros
    rospy.init_node("rssi_scribe")

    args = rospy.myargv(argv=sys.argv)

    if len(args) != 1:
        print "Usage:"
        print "python magic_scribe.py [output filename]"
        exit(1)

    rospy.loginfo("Starting !!!MAGICAL!!! RSSI Scribe")
    rospy.loginfo("If encountering problems with transform, make sure  base link and beacons TF are published and sychronized (refer to https://answers.ros.org/question/123256/sync-ros-bag-timestamps-with-ros-system/ if using rosbag")

    beacons = rospy.get_param('/beacon_localization/map/beacons')

    result = {}

    rospy.sleep(0.1)
    t = tf.TransformListener()
    for beacon in beacons:
        frame = '/beacon_' + beacon['name'] + '_' + beacon['id']
        # rssi = [response.rssi for response in distances_response.measurements if response.bid == beacon['id']][0]
        # print rssi
        t.waitForTransform(frame, '/batman/base_link', rospy.Time(0), rospy.Duration(10.0))
        (trans, r) = t.lookupTransform(frame, '/batman/base_link', rospy.Time(0))

        result[beacon['id']] = np.linalg.norm(trans)

    rospy.wait_for_service('/beacon_localization/distances/probabilistic')
    distances_srv = rospy.ServiceProxy('/beacon_localization/distances/probabilistic', GetBeaconDistances)
    distances_response = distances_srv.call()

    for id,dist in result.items():
        res = [response for response in distances_response.measurements if response.bid == id][0]
        try:
            print 'beacon_'+ res.name + '_' + res.bid + '.txt'
            with open('beacon_'+ res.name + '_' + res.bid + '.txt', 'a') as outfile:

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
