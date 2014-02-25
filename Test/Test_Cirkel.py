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
#oh = Oncilla2.OncillaHardware()
oh = None

alpha1 = alpha2 = 15.0*math.pi/180.0
beta1 = beta2 = 135.0*math.pi/180.0
gamma1 = gamma2 = 95.0*math.pi/180.0

alpha3 = alpha4 = 25.0*math.pi/180.0
beta3 = beta4 = 115.0*math.pi/180.0
gamma3 = gamma4 = 100.0*math.pi/180.0

q_start = numpy.array([alpha1, beta1, gamma1, alpha2, beta2, gamma2, alpha3, beta3, gamma3, alpha4, beta4, gamma4])

if oh is not None:
  reply = oh.sendConfiguration(q_start)

raw_input("Press key to continue")

speed = 20.0

q = q_start

r = 5.0
xc = 0.0
yc = 0.0

fig = plt.figure()
plt.ion()
plt.show()
plt.axis([-20, 20, -20, 20])
for t in numpy.arange(0.0, 2*math.pi, 0.01):
  x = xc + r*math.cos(t) 
  y = yc + r*math.sin(t) 
  X_goal = numpy.matrix([[20.0],[x],[y]])
  q = OM.Move_COB(oh, X_goal, q, q_start, speed)
  X_current = OK.Relative_COB(q_start, q, 1) 
  plt.plot(X_current.item(1), X_current.item(2), 'ro')
  plt.draw()
  


