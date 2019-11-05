#!/usr/bin/env python3

# Avoid needing display if plots aren't being shown
import sys

import control as ct
# import control as cnt
# import numpy as np
import scipy as sp
import math

import os
import numpy as np

import frccontrol as frccnt

def __make_cost_matrix(elems):
        """Creates a cost matrix from the given vector for use with LQR.

        The cost matrix is constructed using Bryson's rule. The inverse square
        of each element in the input is taken and placed on the cost matrix
        diagonal.

        Keyword arguments:
        elems -- a vector. For a Q matrix, its elements are the maximum allowed
                 excursions of the states from the reference. For an R matrix,
                 its elements are the maximum allowed excursions of the control
                 inputs from no actuation.

        Returns:
        State excursion or control effort cost matrix
        """
        return np.diag(1.0 / np.square(elems))

v = 1e-7

# States: x, y, theta
# Inputs: v, omega

A = np.array([
    [0, 0, 0],
    [0, 0, v],
    [0, 0, 0]
])

B = np.array([
    [1, 0],
    [0, 0],
    [0, 1]
])

C = np.array([
    [0, 0, 1]
])

D = np.array([
    [0, 0]
])

qX = 0.12 # allowable linear error
qY = 0.15 # allowable cross track error
qTheta = math.radians(90) # allowable heading error

vMax = 0.61 * 2.5 # excursion from reference velocity
wMax = math.radians(170.0) # max turn excursion from reference

Q =  __make_cost_matrix([qX, qY, qTheta])
R = __make_cost_matrix([vMax, wMax])

sys = ct.StateSpace(A, B, C, D, remove_useless = False)
sysd = sys.sample(1.0/50.0)

K_0 = frccnt.lqr(sysd, Q, R)

v = 1.0

# States: x, y, theta
# Inputs: v, omega

A = np.array([
    [0, 0, 0],
    [0, 0, v],
    [0, 0, 0]
])

# qX = 0.0625
# qY = 0.125
# qTheta = math.radians(10)

# vMax = 3.96
# wMax = math.radians(140.0)

Q =  __make_cost_matrix([qX, qY, qTheta])
R = __make_cost_matrix([vMax, wMax])

sys = ct.StateSpace(A, B, C, D, remove_useless = False)
sysd = sys.sample(1.0/50.0)

K_1 = frccnt.lqr(sysd, Q, R)

# Kx = K_0 1,1
# Ky_0 = K_0 2,2
# Ky_1 = K_1 2,2
# Ktheta = K_1 2,3

print("\nK0 = \n%s\n " % K_0)
print("K1 = \n%s\n" % K_1)

print("kx = %s" % K_0[1-1,1-1])
print("ky_0 = %s" % K_0[2-1,2-1])
print("ky_1 = %s" % K_1[2-1,2-1])
print("kTheta = %s\n" % K_1[2-1,3-1])

print("%s, %s, %s, %s" % (K_0[1-1,1-1], K_0[2-1,2-1], K_1[2-1,2-1], K_1[2-1,3-1]))

# print(K)

# [[ 3.48624521e+01 -2.36365963e-11  1.68784118e-16]
#  [ 6.42706700e-17  1.90968954e+01  2.27952481e+00]]

# [[3.48624521e+01 1.74507062e-15 3.99678488e-16]
#  [1.45526701e-16 1.82859275e+01 6.42931796e+00]]

# 3.48624521e+01 
# 1.90968954e+01
# 1.82859275e+01
# 6.42931796e+00

# kx = 3.48624521e+01
# ky_0 = 1.70016509e+01
# ky_1 = 1.67766520e+01
# kTheta = 1.33387761e+01
