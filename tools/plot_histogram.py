import numpy as np
import matplotlib.pyplot as plt
import matplotlib.mlab as mlab
import argparse


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('input', help='Input file')
    parser.add_argument('-f', action='store_true',  help='Plot fitted exponent curve: log or exp')
    args = parser.parse_args()

    if not args.input:
        print "Required args not provided. Call with --help parameter to see arguments"
        exit(1)

    data = np.loadtxt(args.input).T

    plt.figure()
    n, bins, patches = plt.hist(data, np.linspace(-90, 0, 90), normed=1, facecolor='green', alpha=0.75)
    y = mlab.normpdf(bins, np.average(data), np.std(data))
    if args.f:
        l = plt.plot(bins, y, 'r--', linewidth=1)
    plt.xlabel('RSSI [dBm]')

    plt.grid()
    plt.show()

