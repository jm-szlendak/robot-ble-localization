import yaml
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.mlab as mlab
import argparse
import os

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('file', help='working dir')

    args = parser.parse_args()

    amcl_file1 = args.file + '-1-amcl.bagy'
    amcl_file2 = args.file + '-2-amcl.bagy'
    beacon_file1 = args.file + '-1.bagy'
    beacon_file2 = args.file + '-2.bagy'
    actual_file1 = args.file + '-1-actual.bagy'
    actual_file2 = args.file + '-2-actual.bagy'

    with \
            open(amcl_file1, "r") as amcl1, \
            open(amcl_file2, "r") as amcl2, \
            open(beacon_file1, "r") as beacon1, \
            open(beacon_file2, "r") as beacon2, \
            open(actual_file1, "r") as actual1, \
            open(actual_file2, "r") as actual2:

        measurements_beacon1 = [yaml.load(x) for x in beacon1.read().split('---')]
        measurements_beacon1.pop()
        measurements_beacon2 = [yaml.load(x) for x in beacon2.read().split('---')]
        measurements_beacon2.pop()
        measurements_amcl1 = [yaml.load(x) for x in amcl1.read().split('---')]
        measurements_amcl1.pop()
        measurements_amcl2 = [yaml.load(x) for x in amcl2.read().split('---')]
        measurements_amcl2.pop()

        actual_pose1 = yaml.load(actual1.read())
        actual_pose2 = yaml.load(actual2.read())

        points_beacon1 = [[x['pose']['position']['x'], x['pose']['position']['y']] for x in measurements_beacon1]
        points_amcl1= [[x['pose']['pose']['position']['x'], x['pose']['pose']['position']['y']] for x in measurements_amcl1]
        point_actual1 = [actual_pose1['x'], actual_pose1['y']]
        points_beacon2 = [[x['pose']['position']['x'], x['pose']['position']['y']] for x in measurements_beacon2]
        points_amcl2 = [[x['pose']['pose']['position']['x'], x['pose']['pose']['position']['y']] for x in measurements_amcl2]
        point_actual2 = [actual_pose2['x'], actual_pose2['y']]

        (xb1, yb1) = np.array(points_beacon1).T
        (xa1, ya1) = np.array(points_amcl1).T
        (x1, y1) = np.array(point_actual1).T
        (xb2, yb2) = np.array(points_beacon2).T
        (xa2, ya2) = np.array(points_amcl2).T
        (x2, y2) = np.array(point_actual2).T

        plt.figure(0)
        plt.axis([-2, 5, -3, 3])
        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.scatter(xb1, yb1, c='green')
        plt.scatter(xa1, ya1, marker='+', s=100, c='green')
        plt.scatter(x1, y1, marker='x', s=100, c='green')

        plt.scatter(xb2, yb2, c='red')
        plt.scatter(xa2, ya2, marker='+', s=100, linewidths=2, c='red')
        plt.scatter(x2, y2, marker='x', s=100, linewidths=2, c='red')
        plt.grid()


        plt.savefig(args.file+'.png')
        plt.close()


