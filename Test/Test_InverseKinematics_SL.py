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

q_ref = numpy.array([0.0, 135.0, 90.0, 
                     0.0, 135.0, 90.0, 
                     0.0, 135.0, 90.0, 
                     0.0, 135.0, 90.0])*math.pi/180.0

leg = int(sys.argv[1])

print 'Leg: ', leg

print 'Foot Positions:'
print OK.RelativeFootPositions(q_ref)

X_goal = numpy.array([[float(sys.argv[2])], [float(sys.argv[3])],
  [float(sys.argv[4])]])

print 'Goal:\t', X_goal.T

print OK.InverseKinematics_SL(q_ref[(leg-1)*3:3*(leg-1)+3], X_goal, leg)
