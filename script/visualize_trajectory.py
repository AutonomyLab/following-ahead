#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import sys

if len(sys.argv) <= 1:
	print("usage: python visualize_trajectory.py <trajectory_filename>")
	exit(0)

filename = sys.argv[1]
data = np.loadtxt(filename)


fig = plt.figure()
ax = fig.add_subplot(111)

ax.plot(data[:, 0], data[:, 1], label="robot trajectory")
ax.plot(data[:, 2], data[:, 3], label="person trajectory")

ax.legend()
ax.set_xlabel("x [m]")
ax.set_xlabel("y [m]")

plt.savefig("trajectory.jpg")