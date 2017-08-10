#!/usr/bin/env python

from math import sin, cos, atan2, pi
import numpy as np
from sys import argv
import cv2

def rotation2D(angle):
	angle = angle * pi / 180
	return np.asarray([
		[cos(angle), -sin(angle)],
		[sin(angle), cos(angle)],
	])

def oppositeAngle(angle):
	angle = angle * pi / 180.0
	unit_vector1 = [cos(angle), sin(angle)]
	unit_vector2 = [-unit_vector1[0], -unit_vector1[1]]
	new_angle = atan2(unit_vector2[1], unit_vector2[0])
	return new_angle * 180.0 / pi

def rot2ang(R):
	return atan2(R[1, 0], R[1, 1]) * 180.0 / pi

if len(argv) < 2:
	print("usage: python direction_selection.py <theta_rp> <theta_ol>")

theta_rp = float(argv[1])
theta_ol = float(argv[2])
theta_ol2 = oppositeAngle(theta_ol)

R_w_rp = rotation2D(theta_rp)
R_w_vl1 = rotation2D(theta_ol)
R_w_vl2 = rotation2D(theta_ol2)

R_rp_vl1 = np.dot(R_w_rp.transpose(), R_w_vl1)
R_rp_vl2 = np.dot(R_w_rp.transpose(), R_w_vl2)

angle1 = rot2ang(R_rp_vl1)
angle2 = rot2ang(R_rp_vl2)

print "angle1: ", angle1
print "angle2: ", angle2

print "angle1" if (abs(angle1) < abs(angle2)) else "angle2"

img = np.zeros((100, 100, 3), dtype=np.uint8)

def drawVector(img, angle, color):
	angle = angle * pi / 180.0
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

drawVector(img, theta_rp, (0, 255, 0))
drawVector(img, theta_ol, (255, 0, 0))
drawVector(img, theta_ol2, (0, 0, 255))

img = np.flip(img, 0)
cv2.imshow("window", img)
cv2.waitKey()
