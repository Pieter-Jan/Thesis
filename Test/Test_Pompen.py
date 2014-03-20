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

alpha1 = alpha2 = 15.0*math.pi/180.0
beta1 = beta2 = 135.0*math.pi/180.0
gamma1 = gamma2 = 95.0*math.pi/180.0

alpha3 = alpha4 = 25.0*math.pi/180.0
beta3 = beta4 = 115.0*math.pi/180.0
gamma3 = gamma4 = 95.0*math.pi/180.0

q_start = numpy.array([alpha1, beta1, gamma1, alpha2, beta2, gamma2, 
                       alpha3, beta3, gamma3, alpha4, beta4, gamma4])
if oh is not None:
  reply = oh.sendConfiguration(q_start)

raw_input("Press key to continue")

speed = 20.0

q = q_start
for i in range(0, 10):
  X_goal = numpy.matrix([[20.0],[0.0],[0.0]])
  q = OM.MoveCOB(oh, X_goal, q, q_start, speed, None)

  X_goal = numpy.matrix([[0.0],[0.0],[0.0]])
  q = OM.MoveCOB(oh, X_goal, q, q_start, speed, None)
