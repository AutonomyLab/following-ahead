#!/usr/bin/env python

from math import sin, cos, acos, atan2, pi
import numpy as np
from sys import argv
import cv2

def rotation2D(angle):
	return np.asarray([
		[cos(angle), -sin(angle)],
		[sin(angle), cos(angle)],
	])

def oppositeAngle(angle):
	unit_vector1 = [cos(angle), sin(angle)]
	unit_vector2 = [-unit_vector1[0], -unit_vector1[1]]
	new_angle = atan2(unit_vector2[1], unit_vector2[0])
	return new_angle

def rot2ang(R):
	return atan2(R[1, 0], R[1, 1])

def deg2rad(angle):
	return angle * pi / 180.0

if len(argv) < 2:
	print("usage: python direction_selection.py <theta_rp> <theta_ol>")

theta_rp = deg2rad(float(argv[1]))
theta_ol = deg2rad(float(argv[2]))
theta_ol2 = oppositeAngle(theta_ol)

object_vector = np.asarray([
    cos(theta_rp),
    sin(theta_rp)
])

object_vector /= np.linalg.norm(object_vector)

# two angles of the obstacle line
ol_theta = [0, 0]
ol_theta[0] = theta_ol
ol_theta[1] = oppositeAngle(ol_theta[0])

# vector normal to the obstacle (needed for backing off)
obstacle_normal_vectors = [None, None]
obstacle_normal_vectors[0] = np.asarray([-sin(ol_theta[0]), cos(ol_theta[0])])
obstacle_normal_vectors[1] = np.asarray([sin(ol_theta[0]), -cos(ol_theta[0])])


object2obstacle_normal_angle = [
	acos(obstacle_normal_vectors[0].dot(object_vector)),
	acos(obstacle_normal_vectors[1].dot(object_vector))
]

obstacle_normal_angles = [
	atan2(obstacle_normal_vectors[0][1], obstacle_normal_vectors[0][0]),
	atan2(obstacle_normal_vectors[1][1], obstacle_normal_vectors[1][0])
]

print "normals: ", [i * 180 / pi for i in obstacle_normal_angles]
# choose max cuz you don't want to go through the obstacle
backing_off_angle = obstacle_normal_angles[0] if (object2obstacle_normal_angle[0] > object2obstacle_normal_angle[1]) \
					else obstacle_normal_angles[1]

img = np.zeros((100, 100, 3), dtype=np.uint8)

def drawVector(img, angle, color):
	cv2.line(
		img,
		(50,50),
		(
			int(round(50 + 100 * cos(angle))),
			int(round(50 + 100 * sin(angle)))
		),
		color,
		5
	)

drawVector(img, theta_rp, (255, 255, 255))
drawVector(img, theta_ol, (255, 0, 0))
drawVector(img, backing_off_angle, (0, 0, 255))

img = np.flip(img, 0)
cv2.imshow("window", img)
cv2.waitKey()
