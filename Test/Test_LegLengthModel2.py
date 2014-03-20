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

# Start configuration
alpha1 = alpha2 = 0.0*math.pi/180.0
beta1 = beta2 = 135.0*math.pi/180.0
gamma1 = gamma2 = 90.0*math.pi/180.0

alpha3 = alpha4 = 0.0*math.pi/180.0
beta3 = beta4 = 115.0*math.pi/180.0
gamma3 = gamma4 = 90.0*math.pi/180.0

q_start = numpy.array([alpha1, beta1, gamma1, alpha2, beta2, gamma2, 
                       alpha3, beta3, gamma3, alpha4, beta4, gamma4])
if oh is not None:
  reply = oh.sendConfiguration(q_start)

oh.swingLeg = 4

# Get unit vector in direction of leg 1
leg = 1
shoulder1 = numpy.array([[OK.Qs[leg-1]], [OK.Rs[leg-1]], [OK.Ss[leg-1]]])
foot1 = OK.FootPositions_FromServo(alpha1, beta1, gamma1, leg)
u1 = (foot1 - shoulder1)/numpy.linalg.norm(foot1 - shoulder1)

# Get unit vector in direction of leg 2
leg = 2
shoulder2 = numpy.array([[OK.Qs[leg-1]], [OK.Rs[leg-1]], [OK.Ss[leg-1]]])
foot2 = OK.FootPositions_FromServo(alpha2, beta2, gamma2, leg)
u2 = (foot2 - shoulder2)/numpy.linalg.norm(foot2 - shoulder2)

qNew = q_start

step = 20.0
step = -step
rel_feet = OK.RelativeFootPositions(q_start)

leg = 1
pNew1 = rel_feet[:,leg-1] + u1*step
qNew_leg1 = OK.InverseKinematics_SL(q_start[3*(leg-1):3*(leg-1)+3], pNew1, leg)
qNew[3*(leg-1):3*(leg-1)+3] = qNew_leg1

leg = 2
pNew2 = rel_feet[:,leg-1] + u2*step
qNew_leg2 = OK.InverseKinematics_SL(q_start[3*(leg-1):3*(leg-1)+3], pNew2, leg)
qNew[3*(leg-1):3*(leg-1)+3] = qNew_leg2

feet1 = OK.RelativeFootPositions(q_start)
feet2 = OK.RelativeFootPositions(qNew)
leg = 1
print OK.FootPositions_FromServo(qNew[3*(leg-1)], qNew[3*(leg-1) + 1], qNew[3*(leg-1) + 2], leg)
print numpy.linalg.norm(feet1[:,0])
leg = 2
print OK.FootPositions_FromServo(qNew[3*(leg-1)], qNew[3*(leg-1) + 1], qNew[3*(leg-1) + 2], leg)
print numpy.linalg.norm(feet2[:,0])

reply = oh.sendConfiguration(qNew)

print OK.Relative_COB(q_start, qNew, 4)

while True:
    None


