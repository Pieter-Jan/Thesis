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

reply = oh.sendConfiguration(q_start)

raw_input("Press key to continue")

print OK.RelativeFootPositions(q_start) 


