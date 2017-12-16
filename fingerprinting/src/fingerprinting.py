#!/usr/bin/env python
import sys
import os.path

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from beacon_msgs.msg import LocationTag, BeaconsScan, BeaconPositionAndDistance
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion, Vector3, \
    PoseWithCovariance
from std_msgs.msg import Header
from beacon_msgs.srv import GetBeaconDistances
from pprint import PrettyPrinter
import rospy
import yaml
import numpy as np
refpoints = None
ref_vector_dimension = 0
pp = PrettyPrinter(indent=2)
pub = None
def callback(msg):
    print 'cb'
    if not refpoints:
        return
    distribution = []
    for point in refpoints:
        squared_dist = 0.0
        for beacon in msg.beacons:
            squared_dist += (beacon.rssi - point['rssi'][beacon.name])**2
        dist = np.sqrt(squared_dist)

        measurement = {
            'x': point['x'],
            'y': point['y'],
            'dist': dist,
        }

        distribution.append(measurement)

    distance_sum = sum(item['dist'] for item in distribution)

    mindist = 1.0
    minindex = 0
    # unconstrained
    for index, item in enumerate(distribution):
        item['dist_normalized'] = item['dist']/distance_sum
        if item['dist_normalized'] < mindist:
            mindist = item['dist_normalized']
            minindex = index

    # avg_x = sum(item['x']*item['dist_normalized'] for item in distribution)/sum(item['dist_normalized'] for item in distribution)
    # avg_y = sum(item['y']*item['dist_normalized'] for item in distribution)/sum(item['dist_normalized'] for item in distribution)
    # print  avg_x, avg_y
    pose = PoseWithCovarianceStamped(
            header=Header(
                stamp=rospy.Time.now(),
                frame_id='map'
            ),
            pose=PoseWithCovariance(
                pose=Pose(
                    position=Point(
                        # x=avg_x,
                        # y=avg_y
                        x=distribution[minindex]['x'],
                        y=distribution[minindex]['y']
                    ),
                    orientation=Quaternion(
                        x=0.707,
                        y=0,
                        z=0,
                        w=0.707)),
                covariance=np.zeros((6, 6)).flatten()
            )
        )
    pub.publish(pose)




if __name__ == "__main__":
    # init ros
    rospy.init_node("fingerprinting")
    rospy.loginfo("Starting Fingerprinting node")
    args = rospy.myargv(argv=sys.argv)

    pub = rospy.Publisher('beacon_localization/bl_pose', PoseWithCovarianceStamped, queue_size=100)

    with open(args[1], 'r') as map_file:
        refpoints = [ yaml.load(x) for x in map_file.read().split('---')[:-1]]
        ref_vector_dimension = len(refpoints[0]['rssi'].keys())

        rospy.Subscriber('beacon_localization/distances/probabilistic', BeaconsScan, callback)

        #
        #
        # x = np.linspace(-10, 10, 20/0.05+1)
        # y = np.linspace(-10, 10, 20/0.05+1)
        # cunt = 0
        # for x in np.linspace(-10, 10, 20/0.5+1):
        #     for y in np.linspace(-10, 10, 20/0.1+1):
        #         print x, y
        #         cunt += 1
        # print cunt


    rospy.spin()
