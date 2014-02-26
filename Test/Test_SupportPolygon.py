import sys
sys.path.append('../Oncilla')

import OncillaKinematics as OK
import OncillaMovement as OM
import OncillaVisualization as OV
import math
import numpy
import matplotlib.pyplot as plt

alpha1 = 15.0*math.pi/180.0
beta1 = 135.0*math.pi/180.0
gamma1 = 100.0*math.pi/180.0

alpha2 = 15.0*math.pi/180.0
beta2 = 135.0*math.pi/180.0
gamma2 = 100.0*math.pi/180.0

alpha3 = 25.0*math.pi/180.0
beta3 = 115.0*math.pi/180.0
gamma3 = 80.0*math.pi/180.0

alpha4 = 25.0*math.pi/180.0
beta4 = 115.0*math.pi/180.0
gamma4 = 100.0*math.pi/180.0

q_supported = numpy.array([alpha1, beta1, gamma1, alpha2, beta2, gamma2, alpha3, beta3, gamma3, alpha4, beta4, gamma4])

OK.Visualize_SupportPolygon(q_supported, swingLeg=1)

