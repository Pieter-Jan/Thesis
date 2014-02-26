import sys
sys.path.append('../Oncilla')

import OncillaKinematics as OK
import OncillaMovement as OM
import OncillaVisualization as OV
import math
import numpy
import matplotlib.pyplot as plt
import Oncilla2

#alpha1 = alpha2 = alpha3 = alpha4 = 0.0*math.pi/180.0
#beta1 = beta2 = beta3 = beta4 = 135.0*math.pi/180.0
#gamma1 = gamma2 = gamma3 = gamma4 = 90.0*math.pi/180.0
#
#q_start = numpy.array([alpha1, beta1, gamma1, alpha2, beta2, gamma2, alpha3, beta3, gamma3, alpha4, beta4, gamma4])

alpha1 = 25.0*math.pi/180.0
beta1 = 85.0*math.pi/180.0
gamma1 = 90.0*math.pi/180.0

alpha2 = 35.0*math.pi/180.0
beta2 = 145.0*math.pi/180.0
gamma2 = 80.0*math.pi/180.0

alpha3 = 25.0*math.pi/180.0
beta3 = 95.0*math.pi/180.0
gamma3 = 100.0*math.pi/180.0

alpha4 = 25.0*math.pi/180.0
beta4 = 95.0*math.pi/180.0
gamma4 = 80.0*math.pi/180.0

q_start = numpy.array([alpha1, beta1, gamma1, alpha2, beta2, gamma2, alpha3, beta3, gamma3, alpha4, beta4, gamma4])

#OK.Visualize_SupportPolygon(q_start, swingLeg=1)
oh = Oncilla2.OncillaHardware(True)
oh.showSupportPolygon = True
reply = oh.sendConfiguration(q_start)

while True: 
  None
