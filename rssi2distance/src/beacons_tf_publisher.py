#!/usr/bin/env python
import sys
import os.path
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from argparse import ArgumentParser
import rospy
import tf

if __name__ == "__main__":
    # init ros
    rospy.init_node("beacon_tf_publisher")

    robot_name = rospy.get_namespace()
    beacons = rospy.get_param('beacon_localization/map/beacons')
    receiver = rospy.get_param('beacon_localization/receiver')

    br = tf.TransformBroadcaster()
    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        for beacon_cfg in beacons:
            br.sendTransform(
                (beacon_cfg['x'], beacon_cfg['y'], beacon_cfg['z']),
                (0.0, 0.0, 0.0, 1.0),
                rospy.Time.now(), 'beacon_'+beacon_cfg['name'] + '_' + beacon_cfg['id'],
                'map'
            )

        br.sendTransform(
            (receiver['x'], receiver['y'], receiver['z']),
            (0.0, 0.0, 0.0, 1.0),
            rospy.Time.now(), robot_name+'radio_base_link',
            robot_name+'base_link'

        )
        rate.sleep()

