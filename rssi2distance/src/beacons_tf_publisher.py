#!/usr/bin/env python
import sys
import os.path
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))


import rospy
import tf

if __name__ == "__main__":
    # init ros
    rospy.init_node("beacon_tf_publisher")

    beacons = rospy.get_param('/beacon_localization/map/beacons')

    br = tf.TransformBroadcaster()
    rate = rospy.Rate(1.0)

    while not rospy.is_shutdown():
        for beacon_cfg in beacons:
            br.sendTransform(
                (beacon_cfg['x'], beacon_cfg['y'], beacon_cfg['z']),
                (0.0, 0.0, 0.0, 1.0),
                rospy.Time.now(), 'beacon_'+beacon_cfg['name'] + '_' + beacon_cfg['id'],
                'world'
            )
        rate.sleep()

