#!/usr/bin/env python
import sys
import os.path

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from beacon_msgs.msg import LocationTag, BeaconsScan, BeaconPositionAndDistance
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion, Vector3, \
    PoseWithCovariance
from std_msgs.msg import Header
from functools import reduce
from beacon_msgs.srv import GetBeaconDistances
from pprint import PrettyPrinter
import rospy
import yaml
import numpy as np

refpoints = None
ref_vector_dimension = 0
pp = PrettyPrinter(indent=2)
pub = None

def publish_max(distribution):
    distance_sum = sum(item['dist'] for item in distribution)
    mindist = 1.0
    minindex = 0
    for index, item in enumerate(distribution):
        item['dist_normalized'] = item['dist']/distance_sum
        if item['dist_normalized'] < mindist:
            mindist = item['dist_normalized']
            minindex = index

    pose = PoseWithCovarianceStamped(
            header=Header(
                stamp=rospy.Time.now(),
                frame_id='map'
            ),
            pose=PoseWithCovariance(
                pose=Pose(
                    position=Point(
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

def publish_maxavg(distribution):
    sorted_dist = sorted(distribution[:], key=lambda x:x['dist'])
    print sorted_dist[:1]
    avg_pose = {'x':0, 'y':0}
    selected_ones = sorted_dist[:1]
    for measurement in selected_ones:
        avg_pose['x'] += measurement['x']*measurement['dist']
        avg_pose['y'] += measurement['y'] * measurement['dist']

    avg_pose['x'] /= sum(item['dist'] for item in selected_ones)
    avg_pose['y'] /= sum(item['dist'] for item in selected_ones)

    pose = Pose(
        position=Point(
            x=avg_pose['x'],
            y=avg_pose['y']
        ),
        orientation=Quaternion(
            x=0.707,
            y=0,
            z=0,
            w=0.707))
    posewithcovar = PoseWithCovarianceStamped(
            header=Header(
                stamp=rospy.Time.now(),
                frame_id='map'
            ),
            pose=PoseWithCovariance(
                pose=pose,
                covariance=np.zeros((6, 6)).flatten()
            )
        )
    pub.publish(posewithcovar)
    pub_array.publish(PoseArray(
        header=Header(
            stamp=rospy.Time.now(),
            frame_id='map'
        ),
        poses=[Pose(
            position=Point(
                x=p['x'],
                y=p['y']
            ),
            orientation=Quaternion(
                x=0.707,
                y=0,
                z=0,
                w=0.707)) for p in selected_ones]
    ))

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

    publish_maxavg(distribution)



if __name__ == "__main__":
    # init ros
    rospy.init_node("fingerprinting")
    rospy.loginfo("Starting Fingerprinting node")
    args = rospy.myargv(argv=sys.argv)

    pub = rospy.Publisher('beacon_localization/bl_pose', PoseWithCovarianceStamped, queue_size=100)
    pub_array = rospy.Publisher('beacon_localization/bl_pose_array', PoseArray, queue_size=100)

    with open(args[1], 'r') as map_file:
        refpoints = [ yaml.load(x) for x in map_file.read().split('---')[:-1]]
        ref_vector_dimension = len(refpoints[0]['rssi'].keys())
        rospy.Subscriber('beacon_localization/distances/probabilistic', BeaconsScan, callback)
        rospy.spin()
