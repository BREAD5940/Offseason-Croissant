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
qY = 0.2 # allowable cross track error
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

Q =  __make_cost_matrix([qX, qY, qTheta])
R = __make_cost_matrix([vMax, wMax])

sys = ct.StateSpace(A, B, C, D, remove_useless = False)
sysd = sys.sample(1.0/50.0)

K_1 = frccnt.lqr(sysd, Q, R)

print("\nK0 = \n%s\n " % K_0)
print("K1 = \n%s\n" % K_1)

print("kx = %s" % K_0[1-1,1-1])
print("ky_0 = %s" % K_0[2-1,2-1])
print("ky_1 = %s" % K_1[2-1,2-1])
print("kTheta = %s\n" % K_1[2-1,3-1])

gains = ("%s, %s, %s, %s" % (K_0[1-1,1-1], K_0[2-1,2-1], K_1[2-1,2-1], K_1[2-1,3-1]))
print(gains)

# Save it to a file
fileName = "LTVUnicycleGains.kt"
file = open("src/main/kotlin/frc/robot/subsystems/drive/" + fileName, "w")
file.write(
    "package frc.robot.subsystems.drive\n\n" + 
    "import org.team5940.pantry.lib.LTVUnicycleController\n\n" +
    "val optimalLTVUnicycleController get() = LTVUnicycleController(\n" +
    "    %s\n" % gains +
    ") "
)
