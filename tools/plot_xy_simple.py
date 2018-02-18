import yaml
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.mlab as mlab
import argparse
import os

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('beacon', help='working dir')
    parser.add_argument('amcl', help='Input file pattern')
    parser.add_argument('actual', help='Input file pattern')

    args = parser.parse_args()

    with open(args.beacon, "r") as beacon_file, open(args.amcl, "r") as amcl_file, open(args.actual, "r") as actual_file:
        measurements_beacon = [yaml.load(x) for x in beacon_file.read().split('---')]
        measurements_beacon.pop()
        measurements_amcl = [yaml.load(x) for x in amcl_file.read().split('---')]
        measurements_amcl.pop()

        actual_pose = yaml.load(actual_file.read())

        points_beacon = [[x['pose']['position']['x'], x['pose']['position']['y']] for x in measurements_beacon]
        points_amcl= [[x['pose']['pose']['position']['x'], x['pose']['pose']['position']['y']] for x in measurements_amcl]
        point_actual = [actual_pose['x'], actual_pose['y']]

        (x, y) = np.array(points_beacon).T
        (x2, y2) = np.array(points_amcl).T
        (x3, y3) = np.array(point_actual).T
        plt.figure(0)
        plt.axis([-2, 5, -3, 3])
        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.scatter(x, y)
        plt.scatter(x2, y2, marker='+', s=100, c='red')
        plt.scatter(x3, y3, marker='x', s=100, c='green')
        plt.grid()


        plt.savefig(args.beacon+'.png')
        plt.close()


