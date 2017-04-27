import numpy as np
import matplotlib.pyplot as plt
import argparse
from scipy.optimize import curve_fit


def funcL(x, a, b, c):
    return -a * np.log(b * x) + c


def funcE(x, a, b, c):
    return a * np.exp(-b * x) + c

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('input', help='Input file')
    parser.add_argument('--log', action='store_true', help='Log X scale')
    parser.add_argument('-f', help='Plot fitted exponent curve: log or exp')
    args = parser.parse_args()

    if not args.input:
        print "Required args not provided. Call with --help parameter to see arguments"
        exit(1)

    data = np.loadtxt(args.input).T
    x = data[0]
    y = data[1]
    yerr = data[2]
    xlogfit, _ = curve_fit(funcL, x, y, sigma=yerr)
    xexpfit, _= curve_fit(funcE, x, y, sigma=yerr)
    print 'Log fitting results: [a,b,c]: ', xlogfit
    print 'Exp fitting results: [a,b,c]: ', xexpfit

    if args.log:
        plt.subplot(111, xscale="log")
    else:
        plt.subplot(111)

    plt.errorbar(x, y, yerr, fmt='x')

    if args.f == 'exp':
        plt.plot(x, funcE(x, *xexpfit))
    if args.f == 'log':
        plt.plot(x, funcL(x, *xlogfit))
    if args.f == 'both':
        plt.plot(x, funcL(x, *xlogfit))
        plt.plot(x, funcE(x, *xexpfit))
    plt.axis([0, x[-1] + 0.1 * x[-1], 1.2 * np.amin(y), 5])
    plt.xlabel('Odleglosc [m]')
    plt.ylabel('RSSI [dBm]')
    plt.grid()
    plt.show()
    # print data.T
