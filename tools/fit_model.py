import numpy as np
import matplotlib.pyplot as plt
import argparse
from scipy.optimize import curve_fit


def funcL(x, a, b, c):
    return -a * 10 * np.log10(-x)

def funcP(x, a, b):
    return 10 ** ((x - a)/b)


def funcE(x, a, b, c):
    return a * np.exp( x) + c


def interpolate(measures):

    log_d1 = np.log10(measures[0][1])
    log_d2 = np.log10(measures[1][1])

    P1 = measures[0][0]
    P2 = measures[1][0]

    print log_d1, log_d2, P1, P2

    A = (P1-P2)/(log_d1 - log_d2)
    B = P2 - ((P1-P2)*log_d2)/(log_d1-log_d2)
    return [B, A]

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
    y = data[0]
    x = data[1]



    # xlogfit, _ = curve_fit(funcL, x, y)
    # xexpfit, _ = curve_fit(funcL, x, y)
    xpowfit, _ = curve_fit(funcP, -x, y, maxfev=6000)
    xpow_interpolation = interpolate([(-x[0], y[0]), (-x[-1], y[-1])])


    print 'Pow fitting results: [a,b,c]: ', xpowfit
    print 'Interpolation result: [a, b]: ', xpow_interpolation
    # print 'Log fitting results: [a,b,c]: ', xlogfit
    # print 'Exp fitting results: [a,b,c]: ', xexpfit

    if args.log:
        plt.subplot(111, xscale="log")
    else:
        plt.subplot(111)

    plt.scatter(x, y)
    # print funcL(np.linspace(0,100,100), *xlogfit)
    # plt.plot(x, funcL(x, *xlogfit))
    plt.plot(-np.linspace(0,100,100), funcP(np.linspace(0,100,100), *xpowfit), label='Dopasowanie krzywej')
    plt.plot(-np.linspace(0,100,100), funcP(np.linspace(0,100,100), *xpow_interpolation), label='Interpolacja')

    # plt.plot(x, funcE(x, *xexpfit))
        # plt.plot(x, funcL(x, *xlogfit))
    plt.axis([0, x[-1] + 0.1 * x[-1], 1.2 * np.amin(y), 5])
    plt.ylabel('Odleglosc [m]')
    plt.xlabel('RSSI [dBm]')
    plt.legend()
    plt.grid()
    plt.show()
    # print data.T
