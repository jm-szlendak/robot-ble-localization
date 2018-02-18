import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import numpy as np
from scipy.stats import multivariate_normal

xl = np.linspace(-6, 6, 480)
yl = np.linspace(-6, 6, 480)

x, y = np.meshgrid(xl, yl)

# Need an (N, 2) array of (x, y) pairs.
xy = np.column_stack([x.flat, y.flat])

mu1 = np.array([3.0, 3.0])
mu2 = np.array([-3.0, -3.0])
mu3 = np.array([3.0, -3.0])
mu4 = np.array([-3.0, 3.0])

sigma = np.array([.5, .5])
covariance = np.diag(sigma**2)

z1 = multivariate_normal.pdf(xy, mean=mu1, cov=covariance)
z2 = multivariate_normal.pdf(xy, mean=mu2, cov=covariance)
z3 = multivariate_normal.pdf(xy, mean=mu3, cov=covariance)
z4 = multivariate_normal.pdf(xy, mean=mu4, cov=covariance)
z = (z1+z2+z3+z4)/np.sum(z1+z2+z3+z4)
z = z.reshape(x.shape)
print 'zsum'
print np.sum(z)
fig = plt.figure()

ax = fig.add_subplot(111, projection='3d')
ax.plot_surface(x,y,z, color='green', linewidth=0.5)
ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
ax.tick_params(axis='z', which='major', pad=15)
plt.savefig('laser.png', dpi=200)


#beacons

mu_b = np.array([2, 2])
sigma_b = np.array([3, 3])
covar_b = np.diag(sigma_b**2)
z_b = multivariate_normal.pdf(xy, mean=mu_b, cov=covar_b)
z_b /= np.sum(z_b)
z_b = z_b.reshape(x.shape)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot_surface(x,y,z_b, color='green', linewidth=0.5)
ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
ax.tick_params(axis='z', which='major', pad=15)
plt.savefig('ble.png', dpi=200)


#sum
z_sum = np.multiply(z, z_b)

nsum = np.sum(z_sum)
print nsum
z_sum = z_sum/ nsum

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot_surface(x,y,z_sum, color='green', linewidth=0.5)
ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
ax.tick_params(axis='z', which='major', pad=15)
plt.savefig('sum.png', dpi=200)


plt.figure()
plt.axis([-6, 6, -6, 6])
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.grid()
a,b = np.array([[3,3], [-3,-3], [3, -3], [-3, 3]]).T
plt.scatter(a,b, marker='+', s=100, c='red')
plt.scatter(2, 2, s=100)
plt.savefig('2d.png', dpi=200)
