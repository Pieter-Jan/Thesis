import sys
sys.path.append('../Oncilla')

import OncillaKinematics as OK
import OncillaMovement as OM
import numpy
import math
import Oncilla2
import time
numpy.set_printoptions(linewidth=400)
numpy.set_printoptions(threshold=numpy.nan)
numpy.set_printoptions(precision=4)

# Oncilla hardware object
oh = Oncilla2.OncillaHardware()

alpha1 = alpha2 = alpha3 = alpha4 = 0.0*math.pi/180.0
beta1 = beta2 = beta3 = beta4 = 135.0*math.pi/180.0
gamma1 = gamma2 = gamma3 = gamma4 = 90.0*math.pi/180.0

q_start = numpy.array([alpha1, beta1, gamma1, alpha2, beta2, gamma2, alpha3, beta3, gamma3, alpha4, beta4, gamma4])
reply = oh.sendConfiguration(q_start)

raw_input("Press key to continue")

speed = 30.0

# Move Alpha of leg 1:
alpha1 = -10.0*math.pi/180.0
q1 = numpy.array([alpha1, beta1, gamma1, alpha2, beta2, gamma2, alpha3, beta3, gamma3, alpha4, beta4, gamma4])
reply = oh.sendConfiguration(q1)

# Move Servo of leg 1:
gamma1 = 100.0*math.pi/180.0
q2 = numpy.array([alpha1, beta1, gamma1, alpha2, beta2, gamma2, alpha3, beta3, gamma3, alpha4, beta4, gamma4])
reply = oh.sendConfiguration(q2)
