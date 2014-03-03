import sys
sys.path.append('../Oncilla')

import OncillaKinematics as OK
import OncillaMovement as OM
import numpy
import math
import Oncilla2
import time
import matplotlib.pyplot as plt
numpy.set_printoptions(linewidth=400)
numpy.set_printoptions(threshold=numpy.nan)
numpy.set_printoptions(precision=4)

# Oncilla hardware object
oh = Oncilla2.OncillaHardware(True)
oh.showSupportPolygon = True

q_start = numpy.array([0.0, 135.0, 90.0, 
                       0.0, 135.0, 90.0, 
                       0.0, 135.0, 90.0, 
                       0.0, 135.0, 90.0])*math.pi/180.0

if oh is not None:
  reply = oh.sendConfiguration(q_start)

raw_input("Press key to continue")

speed = 20.0

X_goal = numpy.matrix([[30],[0.0],[0.0]])
q = OM.Move_COB(oh, X_goal, q_start, q_start, speed)

r = 20.0
xc = 0.0
yc = 0.0

for i in range(0, 10):
  for t in numpy.arange(0.0, 2*math.pi, 0.05):
    x = xc + r*math.cos(t) 
    y = yc + r*math.sin(t) 
    X_goal = numpy.matrix([[30],[x],[y]])
    q = OM.Move_COB(oh, X_goal, q, q_start, speed)
  


