import numpy as np
import matplotlib.pyplot as plt
import argparse


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('input', help='Input file')

    args = parser.parse_args()

    if not args.input:
        print "Required args not provided. Call with --help parameter to see arguments"
        exit(1)

    data = np.loadtxt(args.input).T
    x = data[0]
    y = data[1]
    yerr = data[2]

    plt.figure()
    plt.errorbar(x, y, yerr, fmt='x')
    plt.axis([0, x[-1]+0.1*x[-1], 1.2*np.amin(y), 5])
    plt.xlabel('Odleglosc [m]')
    plt.ylabel('RSSI [dBm]')
    plt.grid()
    plt.show()
    # print data.T

