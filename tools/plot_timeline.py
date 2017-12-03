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
    plt.plot(data)
    plt.axis([0, 100, -80, -45])
    plt.ylabel('RSSI [dBm]')
    plt.xlabel('Czas [s]')

    plt.grid()
    plt.show()

