import yaml
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.mlab as mlab
import argparse
import os

# references = {
#     '1.map3.bagy': (0.2825, 0.804),
#     '2.map3.bagy': (0.4831, 1.6154),
#     '3.map3.bagy': (-0.2194, 1.7537),
#     '4.map3.bagy': (-1.129, 1.8241),
#     '5.map3.bagy': (-1.3584, 0.7917),
#     '6.map3.bagy': (-2.03534, 0.18419),
#     '7.map3.bagy': (-1.2783, -0.39168),
#     '8.map3.bagy': (-0.1141, -0.621919),
#     '9.map3.bagy': (0.2948, -0.14661),
#     '10.map3.bagy': (0.2893, 0.87767),
#     '11.map3.bagy': (1.1677, 1.8098)
# }
references = {
    '1': (0.2825, 0.804),
    '2': (0.4831, 1.6154),
    '3': (-0.2194, 1.7537),
    '4': (-1.129, 1.8241),
    '5': (-1.3584, 0.7917),
    '6': (-2.03534, 0.18419),
    '7': (-1.2783, -0.39168),
    '8': (-0.1141, -0.621919),
    '9': (0.2948, -0.14661),
    '10': (0.2893, 0.87767),
    '11': (1.1677, 1.8098)
}
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('workdir', help='working dir')
    parser.add_argument('input', help='Input file pattern')

    parser.add_argument('-f', action='store_true', help='Plot fitted exponent curve: log or exp')
    args = parser.parse_args()

    print args

    filelist = os.listdir(args.workdir)
    for x in filelist:
        if x.endswith(args.input):
            index = 0
            # try:
            with open(args.workdir + x, "r") as y:
                filename = x
                measurements = [yaml.load(x) for x in y.read().split('---')]
                measurements.pop()
                # print measurements[0]['position']['x']

                if 'header' in measurements[0]:
                    points = [[x['pose']['pose']['position']['x'], x['pose']['pose']['position']['y']] for x in measurements]
                    print points

                    points = [np.average(points, axis=0)]
                    print points
                else:
                    points = [[x['position']['x'], x['position']['y']] for x in measurements]

                (x, y) = np.array(points).T

                plt.figure(index)
                plt.axis([-3, 1.5, -1, 3])
                plt.xlabel('X [m]')
                plt.ylabel('Y [m]')
                plt.scatter(x, y)
                refnum = filename.split('.')[0]
                plt.scatter(references[refnum][0], references[refnum][1], marker='+', s=100, c='red')
                plt.grid()
                # plt.title(filename)
                index += 1

                plt.savefig(args.workdir + filename+'.png')
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
