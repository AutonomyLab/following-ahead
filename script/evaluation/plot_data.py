#!/usr/bin/env python

import rospy
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import sys

FONT_SIZE = 23
'''
data saved in the format 
    timestamp, range, robot_x, robot_y, person_x, person_y
    timestamp, bearing
'''
def showCorrespondingPoint(data, ax):
    font = {
        'family' : 'Times',
        'weight' : 'bold',
        'size'   : FONT_SIZE
    }
    axes = {
        'unicode_minus' : False
    }
    matplotlib.rc('font', **font)
    matplotlib.rc('axes', **axes)

    skip = 80
    for i in range(0, data.shape[0], skip):
        robot_x = data[i][2]
        robot_y = data[i][3]
        person_x = data[i][4]
        person_y = data[i][5]
        ax.plot(
            [robot_x, person_x], [robot_y, person_y], 'c'
        )
        
        markersize = 12
        markercolor_robot = 'r'
        markercolor_person = 'b'
        
        if i == 0:
            markersize = 20
            markercolor_robot = (1.0, 0.5, 0)
            markercolor_person = (0, 0.5, 1.0)
        elif (data.shape[0] - i) < skip:
            markersize = 20
            markercolor_robot = (1.0, 1.0, 0)
            markercolor_person = (0, 1.0, 1.0)

        if i == 0:
            ax.plot([robot_x], [robot_y], color=markercolor_robot, markersize=markersize, marker='o', label="robot trajectory")
            ax.plot([person_x], [person_y], color=markercolor_person, markersize=markersize, marker='^', label="person trajectory")
        else:
            ax.plot([robot_x], [robot_y], color=markercolor_robot, markersize=markersize, marker='o', )
            ax.plot([person_x], [person_y], color=markercolor_person, markersize=markersize, marker='^', )

    ax.legend() # bbox_to_anchor=(1.1, 1.1))
    ax.set_aspect('equal')#, 'datalim')
    ax.set_xticks(np.arange(-3, 3, 0.5))
    ax.set_yticks(np.arange(-3, 3, 0.5))
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")

def showBearing(data, ax):
    font = {
        'family' : 'Times',
        'weight' : 'bold',
        'size'   : FONT_SIZE
    }
    axes = {
        'unicode_minus' : False
    }
    matplotlib.rc('font', **font)
    matplotlib.rc('axes', **axes)
    
    ax.plot(data[:, 0], data[:, 1])
    ax.set_yticks(np.arange(-20, 80, 40))
    ax.set_xlabel("time [sec]")
    ax.set_ylabel("bearing [degree]")

def showRange(data, ax):
    font = {
        'family' : 'Times',
        'weight' : 'bold',
        'size'   : FONT_SIZE
    }
    axes = {
        'unicode_minus' : False
    }
    matplotlib.rc('font', **font)
    matplotlib.rc('axes', **axes)
    
    ax.plot(data[:, 0], data[:, 1])
    ax.set_yticks(np.arange(.2, 2, .4))
    ax.set_xlabel("time [sec]")
    ax.set_ylabel("range [m]")

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print 'Usage: python plot_data.py <file-prefix>'
        exit(0)
    file_prefix= sys.argv[1]
    trajectory_data = np.loadtxt(file_prefix + "_trajectory_data.txt")
    bearing_data = np.loadtxt(file_prefix + "_bearing_data.txt")

    axes = []
    axes.append(plt.subplot2grid((9, 1), (0, 0), rowspan=5))
    axes.append(plt.subplot2grid((9, 1), (6, 0)))
    axes.append(plt.subplot2grid((9, 1), (8, 0)))
    for i in range(len(axes)):
        axes[i].hold(True)
        axes[i].grid(True)
    showCorrespondingPoint(trajectory_data, axes[0])
    showRange(trajectory_data, axes[1])
    showBearing(bearing_data, axes[2])
    plt.show()