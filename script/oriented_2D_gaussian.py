#!/usr/bin/env python

from math import pi, sin, cos, exp
import numpy as np
import cv2

W = 300
H = 300

THETA = 150.0 * pi / 180.0
CENTER_X = 100
CENTER_Y = 150
SIGMA_X = 30
SIGMA_Y = 5

image = np.zeros((W, H), dtype=np.float64)

T = np.asarray([
	[cos(THETA), -sin(THETA), CENTER_X],
	[sin(THETA), cos(THETA), CENTER_Y],
	[0.0, 0.0, 1.0],
])

T_inv = np.linalg.inv(T)

for y in range(H):
	for x in range(W):
		# change the reference frame of the point to the gaussian's
		p_local = np.dot(T_inv, np.asarray([[x, y, 1]]).transpose())

		image[H-1 - y, x] = 1000.0/(2*pi*SIGMA_X*SIGMA_Y) * \
			exp(- \
				(
					pow(p_local[0, 0], 2)/(2*pow(SIGMA_X, 2)) + \
					pow(p_local[1, 0], 2)/(2*pow(SIGMA_Y, 2)) \
				) \
			);

cv2.imshow("window", image)
cv2.waitKey()



