import yaml
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.mlab as mlab
import argparse
import os

references = np.array([
    [0.2825, 0.804],
    [0.4831, 1.6154],
    [-0.2194, 1.7537],
    [-1.129, 1.8241],
    [-1.3584, 0.7917],
    [-2.03534, 0.18419],
    [-1.2783, -0.39168],
    [-0.1141, -0.621919],
    [0.2948, -0.14661],
    [0.2893, 0.87767],
    [1.1677, 1.8098]
]).T
if __name__ == '__main__':
    plt.figure()
    plt.axis([-3, 1.5, -1, 3])
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.scatter(references[0], references[1], marker='+', s=100, c='red')
    for i, refpoint in enumerate(references.T):
        plt.annotate(str(i+1), (refpoint[0], refpoint[1]), xytext=(10,0), textcoords='offset points')
    plt.grid()
    plt.savefig('references.png')
    plt.close()

