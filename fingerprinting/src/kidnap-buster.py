#!/usr/bin/env python

""" This is the starter code for the robot localization project """
import sys
import os.path
import rospy
import yaml
from std_msgs.msg import Header, String, ColorRGBA
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion, Vector3
from nav_msgs.srv import GetMap
from visualization_msgs.msg import Marker, MarkerArray
from beacon_msgs.msg import BeaconsScan
from copy import deepcopy

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix
from random import gauss

import math
import random
import time

import numpy as np
from numpy.random import random_sample, normal
from sklearn.neighbors import NearestNeighbors
from occupancy_field import OccupancyField

from helper_functions import (convert_pose_inverse_transform,
                              convert_translation_rotation_to_pose,
                              convert_pose_to_xy_and_theta,
                              angle_diff,sum_vectors)

class Particle(object):
    """ Represents a hypothesis (particle) of the robot's pose consisting of x,y and theta (yaw)
        Attributes:
            x: the x-coordinate of the hypothesis relative to the map frame
            y: the y-coordinate of the hypothesis relative ot the map frame
            theta: the yaw of the hypothesis relative to the map frame
            w: the particle weight (the class does not ensure that particle weights are normalized
    """

    def __init__(self,x=0.0,y=0.0,theta=0.0,w=1.0):
        """ Construct a new Particle
            x: the x-coordinate of the hypothesis relative to the map frame
            y: the y-coordinate of the hypothesis relative ot the map frame
            theta: the yaw of the hypothesis relative to the map frame
            w: the particle weight (the class does not ensure that particle weights are normalized """
        self.w = w
        self.theta = theta
        self.x = x
        self.y = y

    def as_pose(self):
        """ A helper function to convert a particle to a geometry_msgs/Pose message """
        orientation_tuple = tf.transformations.quaternion_from_euler(0,0,self.theta)
        return Pose(position=Point(x=self.x,y=self.y,z=0), orientation=Quaternion(x=orientation_tuple[0], y=orientation_tuple[1], z=orientation_tuple[2], w=orientation_tuple[3]))


class ParticleFilter:
    """ The class that represents a Particle Filter ROS Node
        Attributes list:
            initialized: a Boolean flag to communicate to other class methods that initializaiton is complete
            base_frame: the name of the robot base coordinate frame (should be "base_link" for most robots)
            map_frame: the name of the map coordinate frame (should be "map" in most cases)
            odom_frame: the name of the odometry coordinate frame (should be "odom" in most cases)
            scan_topic: the name of the scan topic to listen to (should be "scan" in most cases)
            n_particles: the number of particles in the filter
            d_thresh: the amount of linear movement before triggering a filter update
            a_thresh: the amount of angular movement before triggering a filter update
            laser_max_distance: the maximum distance to an obstacle we should use in a likelihood calculation
            pose_listener: a subscriber that listens for new approximate pose estimates (i.e. generated through the rviz GUI)
            particle_pub: a publisher for the particle cloud
            laser_subscriber: listens for new scan data on topic self.scan_topic
            tf_listener: listener for coordinate transforms
            tf_broadcaster: broadcaster for coordinate transforms
            particle_cloud: a list of particles representing a probability distribution over robot poses
            current_odom_xy_theta: the pose of the robot in the odometry frame when the last filter update was performed.
                                   The pose is expressed as a list [x,y,theta] (where theta is the yaw)
            map: the map we will be localizing ourselves in.  The map should be of type nav_msgs/OccupancyGrid
    """
    def __init__(self, map_fname):
        self.initialized = False        # make sure we don't perform updates before everything is setup
        rospy.init_node('pf')           # tell roscore that we are creating a new node named "pf"

        self.base_frame = "batman/base_link"  # the frame of the robot base
        self.map_frame = "map"  # the name of the map coordinate frame
        self.odom_frame = "batman/odom"  # the name of the odometry coordinate frame
        self.scan_topic = "batman/scan"  # the topic where we will get laser scans from

        self.n_particles = 500          # the number of particles to use

        self.d_thresh = 0.2             # the amount of linear movement before performing an update
        self.a_thresh = math.pi/6       # the amount of angular movement before performing an update

        self.laser_max_distance = 2.0   # maximum penalty to assess in the likelihood field model

        self.sigma = 0.02                # guess for how inaccurate lidar readings are in meters
        self.beacon_sigma = 2
        # Setup pubs and subs

        # pose_listener responds to selection of a new approximate robot location (for instance using rviz)
        self.pose_listener = rospy.Subscriber("initialpose", PoseWithCovarianceStamped, self.update_initial_pose)
        # publish the current particle cloud.  This enables viewing particles in rviz.
        self.particle_pub = rospy.Publisher("bl_particlecloud", PoseArray, queue_size=10)
        self.marker_pub = rospy.Publisher("markers", MarkerArray, queue_size=10)
        self.pose_pub = rospy.Publisher("/batman/pose", PoseStamped, queue_size=10)
        # laser_subscriber listens for data from the lidar
        self.laser_subscriber = rospy.Subscriber(self.scan_topic, LaserScan, self.scan_received)

        # enable listening for and broadcasting coordinate transforms
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()

        self.particle_cloud = []
        self.laser_pose = None
        self.current_odom_xy_theta = []
        self.cnt = 0
        # request the map from the map server, the map should be of type nav_msgs/OccupancyGrid
        self.map_server = rospy.ServiceProxy('static_map', GetMap)
        self.map = self.map_server().map
        # for now we have commented out the occupancy field initialization until you can successfully fetch the map
        self.occupancy_field = OccupancyField(self.map)
        self.initialize_particle_cloud()

        with open(map_fname, 'r') as map_file:
            self.refpoints = [yaml.load(x) for x in map_file.read().split('---')[:-1]]
            self.ref_vector_dimension = len(self.refpoints[0]['rssi'].keys())
            self.beacons_subscriber = rospy.Subscriber('/batman/beacon_localization/distances/probabilistic', BeaconsScan, self.beacons_received)

        self.initialized = True

    def update_robot_pose(self):
        """ Update the estimate of the robot's pose given the updated particles.
            Computed by taking the weighted average of poses.
        """
        # first make sure that the particle weights are normalized
        self.normalize_particles()

        x = 0
        y = 0
        theta = 0
        angles = []
        for particle in self.particle_cloud:

            x += particle.x * particle.w
            y += particle.y * particle.w
            v = [particle.w * math.cos(math.radians(particle.theta)), particle.w * math.sin(math.radians(particle.theta))]
            angles.append(v)
        theta = sum_vectors(angles)
        orientation_tuple = tf.transformations.quaternion_from_euler(0,0,theta)
        self.robot_pose = Pose(position=Point(x=x,y=y),orientation=Quaternion(x=orientation_tuple[0], y=orientation_tuple[1], z=orientation_tuple[2], w=orientation_tuple[3]))
        self.pose_pub.publish(PoseStamped(
            header=Header(stamp=rospy.Time.now(), frame_id="map"),
            pose=self.robot_pose))

    def update_particles_with_odom(self, msg):
        """ Update the particles using the newly given odometry pose.
            The function computes the value delta which is a tuple (x,y,theta)
            that indicates the change in position and angle between the odometry
            when the particles were last updated and the current odometry.

            msg: this is not really needed to implement this, but is here just in case.
        """
        new_odom_xy_theta = convert_pose_to_xy_and_theta(self.odom_pose.pose)
        # compute the change in x,y,theta since our last update
        if self.current_odom_xy_theta:
            old_odom_xy_theta = self.current_odom_xy_theta
            delta = (new_odom_xy_theta[0] - self.current_odom_xy_theta[0],
                     new_odom_xy_theta[1] - self.current_odom_xy_theta[1],
                     new_odom_xy_theta[2] - self.current_odom_xy_theta[2])

            self.current_odom_xy_theta = new_odom_xy_theta
        else:
            self.current_odom_xy_theta = new_odom_xy_theta
            return

        for particle in self.particle_cloud:
            r1 = math.atan2(delta[1], delta[0]) - old_odom_xy_theta[2]
            d = math.sqrt((delta[0]**2) + (delta[1]**2))

            particle.theta += r1 % 360
            particle.x += d * math.cos(particle.theta) + normal(0,0.1)
            particle.y += d * math.sin(particle.theta) + normal(0,0.1)
            particle.theta += (delta[2] - r1 + normal(0,0.1)) % 360
        # For added difficulty: Implement sample_motion_odometry (Prob Rob p 136)

    def map_calc_range(self,x,y,theta):
        """ Difficulty Level 3: implement a ray tracing likelihood model... Let me know if you are interested """
        pass

    def resample_particles(self):
        """ Resample the particles according to the new particle weights.
            The weights stored with each particle should define the probability that a particular
            particle is selected in the resampling step.  You may want to make use of the given helper
            function draw_random_sample.
        """
        # make sure the distribution is normalized
        self.normalize_particles()

        newParticles = []
        for i in range(len(self.particle_cloud)):
            # resample the same # of particles
            choice = random_sample()
            # all the particle weights sum to 1
            csum = 0 # cumulative sum
            for particle in self.particle_cloud:
                csum += particle.w
                if csum >= choice:
                    # if the random choice fell within the particle's weight
                    newParticles.append(deepcopy(particle))
                    break
        self.particle_cloud = newParticles

    def update_particles_with_laser(self, msg):
        """ Updates the particle weights in response to the scan contained in the msg """
        for particle in self.particle_cloud:
            tot_prob = 0
            for index, scan in enumerate(msg.ranges):
                x,y = self.transform_scan(particle,scan,index)
                # transform scan to view of the particle
                d = self.occupancy_field.get_closest_obstacle_distance(x,y)
                # calculate nearest distance to particle's scan (should be near 0 if it's on robot)
                if not math.isnan(d):
                    tot_prob += math.exp((-d**2)/(2*self.sigma**2))
                # add probability (0 to 1) to total probability

            tot_prob = tot_prob/len(msg.ranges)
            # normalize total probability back to 0-1
            particle.w *= tot_prob
            # assign particles weight

        self.update_robot_pose()

    def transform_scan(self, particle, distance, theta):
        """ Calculates the x and y of a scan from a given particle
        particle: Particle object
        distance: scan distance (from ranges)
        theta: scan angle (range index)
        """
        return (particle.x + distance * math.cos(math.radians(particle.theta + theta)),
                particle.y + distance * math.sin(math.radians(particle.theta + theta)))


    @staticmethod
    def weighted_values(values, probabilities, size):
        """ Return a random sample of size elements from the set values with the specified probabilities
            values: the values to sample from (numpy.ndarray)
            probabilities: the probability of selecting each element in values (numpy.ndarray)
            size: the number of samples
        """
        bins = np.add.accumulate(probabilities)
        return values[np.digitize(random_sample(size), bins)]

    @staticmethod
    def draw_random_sample(choices, probabilities, n):
        """ Return a random sample of n elements from the set choices with the specified probabilities
            choices: the values to sample from represented as a list
            probabilities: the probability of selecting each element in choices represented as a list
            n: the number of samples
        """
        values = np.array(range(len(choices)))
        probs = np.array(probabilities)
        bins = np.add.accumulate(probs)
        inds = values[np.digitize(random_sample(n), bins)]
        samples = []
        for i in inds:
            samples.append(deepcopy(choices[int(i)]))
        return samples

    def update_initial_pose(self, msg):
        """ Callback function to handle re-initializing the particle filter based on a pose estimate.
            These pose estimates could be generated by another ROS Node or could come from the rviz GUI """
        xy_theta = convert_pose_to_xy_and_theta(msg.pose.pose)
        self.initialize_particle_cloud(xy_theta)
        self.fix_map_to_odom_transform(msg)

    def initialize_particle_cloud(self, xy_theta=None):
        """ Initialize the particle cloud.
            Arguments
            xy_theta: a triple consisting of the mean x, y, and theta (yaw) to initialize the
                      particle cloud around.  If this input is ommitted, the odometry will be used """
        self.particle_cloud = []
        nonoccupied_points = self.occupancy_field.free_cells[:]

        for i in range(self.n_particles):
            indx = random.randint(0, len(nonoccupied_points)-1)
            particle = Particle(
                nonoccupied_points[indx]['x'],
                nonoccupied_points[indx]['y'],
                random.random() * 360)
            self.particle_cloud.append(particle)
            del nonoccupied_points[indx]
        self.normalize_particles()
        self.update_robot_pose()
        self.publish_particles()

    def normalize_particles(self):
        """ Make sure the particle weights define a valid distribution (i.e. sum to 1.0) """
        tot_weight = sum([particle.w for particle in self.particle_cloud]) or 1
        for particle in self.particle_cloud:
            particle.w = particle.w/tot_weight;

    def publish_particles(self):
        particles_conv = []
        for p in self.particle_cloud:
            particles_conv.append(p.as_pose())
        # actually send the message so that we can view it in rviz
        self.particle_pub.publish(PoseArray(header=Header(stamp=rospy.Time.now(),
                                            frame_id=self.map_frame),
                                  poses=particles_conv))

        marker_array = []
        for index, particle in enumerate(self.particle_cloud):
            marker = Marker(header=Header(stamp=rospy.Time.now(),
                                          frame_id=self.map_frame),
                                  pose=particle.as_pose(),
                                  type=0,
                                  scale=Vector3(x=particle.w*2,y=particle.w*1,z=particle.w*5),
                                  id=index,
                                  color=ColorRGBA(r=1,a=1))
            marker_array.append(marker)

        self.marker_pub.publish(MarkerArray(markers=marker_array))

    def scan_received(self, msg):
        """ This is the default logic for what to do when processing scan data.
            Feel free to modify this, however, I hope it will provide a good
            guide.  The input msg is an object of type sensor_msgs/LaserScan """
        if not(self.initialized):
            # wait for initialization to complete
            return

        if not self.laser_pose:
            self.tf_listener.waitForTransform(self.base_frame,msg.header.frame_id,rospy.Time(0), rospy.Duration(10.0))
            p = PoseStamped(header=Header(stamp=rospy.Time(0),
                                          frame_id=msg.header.frame_id))
            self.laser_pose = self.tf_listener.transformPose(self.base_frame, p)

        self.recent_scan = msg
        # self.update_particles_with_laser(msg)   # update based on laser scan
        # self.update_robot_pose()                # update robot's pose
        # self.resample_particles()               # resample particles to focus on areas of high density
        # self.publish_particles()

    def beacons_received(self, msg):
        if not self.refpoints:
            return
        distribution = []
        for point in self.refpoints:
            squared_dist = 0.0
            for beacon in msg.beacons:
                squared_dist += (beacon.rssi - point['rssi'][beacon.name]) ** 2
            dist = np.sqrt(squared_dist)

            measurement = {
                'x': point['x'],
                'y': point['y'],
                'dist': dist,
            }

            distribution.append(measurement)

        sorted_dist = sorted(distribution[:], key=lambda x: x['dist'])
        avg_pose = {'x': 0, 'y': 0}
        selected_ones = sorted_dist[:1]
        for measurement in selected_ones:
            avg_pose['x'] += measurement['x'] * measurement['dist']
            avg_pose['y'] += measurement['y'] * measurement['dist']
        avg_pose['x'] /= sum(item['dist'] for item in selected_ones)
        avg_pose['y'] /= sum(item['dist'] for item in selected_ones)


        # Make cloud great again
        for particle in self.particle_cloud:
            dist = math.sqrt( (particle.x - avg_pose['x'])**2 + (particle.y - avg_pose['y'])**2 )
            prob = math.exp((-dist**2)/(2*self.beacon_sigma**2))
            particle.w = prob


        # Create new cloud
        # for i in range(self.n_particles-1):
        #     # initial facing of the particle
        #     theta = random.random() * 360
        #
        #     # compute params to generate x,y in a circle
        #     other_theta = random.random() * 360
        #     radius = random.random() * 1.5
        #     # x => straight ahead
        #     x = radius * math.sin(other_theta) + avg_pose['x']
        #     y = radius * math.cos(other_theta) + avg_pose['y']
        #     particle = Particle(x, y, theta)
        #     self.particle_cloud.append(particle)

        self.normalize_particles()
        if self.cnt is 0:
            self.resample_particles()               # resample particles to focus on areas of high density



            self.cnt = 5

        self.cnt -=1
        self.update_particles_with_laser(self.recent_scan)

        self.update_robot_pose()
        self.publish_particles()

    def fix_map_to_odom_transform(self, msg):
        """ This method constantly updates the offset of the map and
            odometry coordinate systems based on the latest results from
            the localizer """
        (translation, rotation) = convert_pose_inverse_transform(self.robot_pose)
        p = PoseStamped(pose=convert_translation_rotation_to_pose(translation,rotation),
                        header=Header(stamp=rospy.Time(0),frame_id=self.base_frame))
        self.odom_to_map = self.tf_listener.transformPose(self.odom_frame, p)
        (self.translation, self.rotation) = convert_pose_inverse_transform(self.odom_to_map.pose)

    def broadcast_last_transform(self):
        """ Make sure that we are always broadcasting the last map
            to odom transformation.  This is necessary so things like
            move_base can work properly. """
        if not(hasattr(self,'translation') and hasattr(self,'rotation')):
            return
        self.tf_broadcaster.sendTransform(self.translation,
                                          self.rotation,
                                          rospy.get_rostime(),
                                          self.odom_frame,
                                          self.map_frame)

if __name__ == '__main__':
    args = rospy.myargv(argv=sys.argv)
    n = ParticleFilter(args[1])
    r = rospy.Rate(5)
    while not(rospy.is_shutdown()):
        # in the main loop all we do is continuously broadcast the latest map to odom transform
        n.broadcast_last_transform()
        r.sleep()
