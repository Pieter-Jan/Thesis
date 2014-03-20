import sys
sys.path.append('../Oncilla')

import OncillaKinematics as OK
import OncillaMovement as OM
import Oncilla2
import matplotlib.pyplot as plt

import numpy
import math
import time

numpy.set_printoptions(linewidth=400)
numpy.set_printoptions(threshold=numpy.nan)
numpy.set_printoptions(precision=4)

# Oncilla hardware object
oh = Oncilla2.OncillaHardware(simulation=True)
oh.showSupportPolygon = True
oh.fixCenter = True

alpha1 = alpha2 = 0.0*math.pi/180.0
beta1 = beta2 = 135.0*math.pi/180.0
gamma1 = gamma2 = 90.0*math.pi/180.0

alpha3 = alpha4 = 0.0*math.pi/180.0
beta3 = beta4 = 115.0*math.pi/180.0
gamma3 = gamma4 = 90.0*math.pi/180.0

q_start = numpy.array([alpha1, beta1, gamma1, alpha2, beta2, gamma2, alpha3,
    beta3, gamma3, alpha4, beta4, gamma4])

reply = oh.sendConfiguration(q_start)
oh.q_ref = q_start

raw_input("Press key to continue")

speed = 30.0
X_goal = numpy.array([[15], [0.0], [0.0]])
q = OM.MoveCOB(oh, X_goal, q_start, speed, None) 

# print OK.Relative_COB(oh.q_ref, q, 3) 

# Direction of Movement
u = numpy.array([0.0, 1.0, 0.0])

# Move robot with the static gait
q = OM.StaticGait(oh, q, u)

