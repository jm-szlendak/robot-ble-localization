import yaml
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.mlab as mlab
import argparse
import os


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('workdir', help='working dir')
    args = parser.parse_args()

    print args


    with open(args.workdir, "r") as y:

        measurements = [yaml.load(x) for x in y.read().split('---')]
        measurements.pop()
        # print measurements[0]['position']['x']
        points = [ [item['x'], item['y']] for item in measurements ]

        (x, y) = np.array(points).T

        plt.figure()
        plt.axis([-2, 4, -2, 2])
        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.scatter(x, y)
        plt.grid()
        # plt.title(filename)


        plt.savefig(args.workdir + '.png')
        plt.close()

            # except Exception as e:
            #     print e
            #     raise e
    #
    # data = np.loadtxt(args.input).T
    #
    # plt.figure()
    # n, bins, patches = plt.hist(data, np.linspace(-90, 0, 90), normed=1, facecolor='green', alpha=0.75)
    # y = mlab.normpdf(bins, np.average(data), np.std(data))
    # if args.f:
    #     l = plt.plot(bins, y, 'r--', linewidth=1)
    # plt.xlabel('RSSI [dBm]')
    #
    # plt.grid()
    # plt.show()
