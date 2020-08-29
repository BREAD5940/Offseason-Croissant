#!/usr/bin/env python3

# Avoid needing display if plots aren't being shown

import control as ct
# import control as cnt
# import numpy as np
import math

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

# The motor used
motor = frccnt.models.MOTOR_CIM
# Number of motors per side
num_motors = 2.0

# Gearing constants and stuff
motor = frccnt.models.gearbox(motor, num_motors)

# High and low gear ratios of differential drive
Glow = 11.832
Ghigh = 6.095

# Drivetrain mass in kg
m = 130 * 0.45
# Radius of wheels in meters
r = 4.0 * 0.0254
# Radius of robot in meters
rb = 26.0 * 0.0254 / 2.0
# Moment of inertia of the differential drive in kg-m^2
J = 6.0

def calcA(v, isHighGear, C1, C3):
    if(isHighGear):
        G = Ghigh
    else:
        G = Glow

    A = np.array([
        [0, 0, 0, 0.5, 0.5],
        [0, 0, v, 0, 0],
        [0, 0, 0, -1/(2*rb), 1/(2*rb)],
        [0, 0, 0, (1/m + rb * rb / J)*C1, (1/m - rb * rb / J)*C3],
        [0, 0, 0, (1/m - rb * rb / J)*C1, (1/m + rb * rb / J)*C3]
    ])
    return A

def calcB(v, isHighGear, C2, C4):
    if(isHighGear):
        G = Ghigh
    else:
        G = Glow
        
    B = np.array([
        [0, 0],
        [0, 0],
        [0, 0],
        [(1/m + rb * rb / J) * C2, (1/m - rb * rb / J) * C4],
        [(1/m - rb * rb / J) * C2, (1/m + rb * rb / J) * C4]
    ])
    return B

    

def calcGains(isHighGear):

    qX = 0.12 # allowable linear error
    qY = 0.2 # allowable cross track error
    qTheta = math.radians(90) # allowable heading error
    vMax = 5.0 * 0.3048 # allowable velocity error? (ft/sec to meters per sec)
    
    voltMax = 5.0 # max control effort (from trajectory feedforward?)

    # period of the controller
    period = 1.0 / 100.0

    print("Calculating cost matrix for LTV diff drive controller with costs qX %s qY %s qTheta %s vMax %s at %s volts \n" % (qX, qY, qTheta, vMax, voltMax))

    if(isHighGear):
        G = Ghigh
        gear = "High"
    else:
        G = Glow
        gear = "Low"

    C1 = -G ** 2 * motor.Kt / (motor.Kv * motor.R * r ** 2)
    C2 = G * motor.Kt / (motor.R * r)
    C3 = -G ** 2 * motor.Kt / (motor.Kv * motor.R * r ** 2)
    C4 = G * motor.Kt / (motor.R * r)

    Q =  __make_cost_matrix([qX, qY, qTheta, vMax, vMax])
    R = __make_cost_matrix([voltMax, voltMax])

    C = np.array([
        [0, 0, 1, 0, 0],
        [0, 0, 0, 1, 0],
        [0, 0, 0, 0, 1]
    ])

    D = np.array([
        [0, 0],
        [0, 0],
        [0, 0]
    ])

    v = 1e-7
    A = calcA(v, isHighGear, C1, C3)
    B = calcB(v, isHighGear, C2, C4)

    sys = ct.StateSpace(A, B, C, D, remove_useless = False)
    sysd = sys.sample(period)
    K0 = frccnt.lqr(sysd, Q, R)

    v = 1.0
    A = calcA(v, isHighGear, C1, C3)
    B = calcB(v, isHighGear, C2, C4)

    sys = ct.StateSpace(A, B, C, D, remove_useless = False)
    sysd = sys.sample(period)
    K1 = frccnt.lqr(sysd, Q, R)

    kx = K0[0, 0]
    ky_0 = K0[0, 1]
    kvPlus_0 = K0[0, 3]
    kVMinus_0 = K0[1, 3]
    ky_1 = K1[0, 1]
    kTheta_1 = K1[0, 2]
    kVPlus_1 = K1[0, 3]

    gains = str("%s, %s, %s, %s,\n        %s, %s, %s" % (
        kx, ky_0, kvPlus_0, kVMinus_0, ky_1, kTheta_1, kVPlus_1
    ))
    # print((gains))

    print("LTV Diff Drive CTE Controller gains: \n %s" % gains)

    # Save it to a file
    fileName = "GeneratedLTVDiffDriveController.kt"
    
    file = open("src/generated/kotlin/statespace/" + fileName, "w")
    file.write(
        "package statespace\n\n" + 
        "import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics\n" +
        "import org.team5940.pantry.lib.statespace.LTVDiffDriveController\n\n" + 
        "val generatedLTVDiffDriveController" + gear + "Gear get() = LTVDiffDriveController(\n    " + 
        gains + ",\n        DifferentialDriveKinematics(" + str(rb) + ")"+
        ")\n"
    )
