import sys
sys.path.append('../Planning')
sys.path.append('../Oncilla')
import matplotlib.pyplot as plt
import numpy
from matplotlib.patches import Polygon
import OncillaMovement as OM
import OncillaKinematics as OK
import math

q_ref = numpy.array([0.0, 135.0, 90.0, 
                     0.0, 135.0, 90.0, 
                     0.0, 135.0, 90.0, 
                     0.0, 135.0, 90.0])*math.pi/180.0

fig, ax = plt.subplots()

feet = OK.RelativeFootPositions(q_ref)

leg = 3

feet = numpy.delete(feet, 0, axis=0)
supportFeet = numpy.delete(feet, leg-1, axis=1)

P1 = numpy.squeeze(numpy.asarray(supportFeet[:,0]))
P2 = numpy.squeeze(numpy.asarray(supportFeet[:,1]))
P3 = numpy.squeeze(numpy.asarray(supportFeet[:,2]))

P1_acc, P2_acc, P3_acc = OM.ReducedSupportPolygon(q_ref, 10.0, leg)

ax.add_patch(Polygon([P1, P2, P3], closed=True))

margin = 0.5

# Plot support polygon and reduced support polygon
ax.add_patch(Polygon([P1, P2, P3], color='red', closed=True, alpha=0.5))
ax.add_patch(Polygon([P1_acc, P2_acc, P3_acc], color='blue', closed=True,
  alpha=0.5))

# Plot center of front stability line
P_stabCenter = OM.CenterOfFrontStabilityLine(q_ref, leg)
if leg == 3 or leg == 4:
  P_stabCenter = OM.CenterOfLineSegment(P_stabCenter, numpy.array([0.0, 0.0]))

ax.plot(P_stabCenter[0], P_stabCenter[1], 'ro')

plt.axis('equal')
plt.axis([-100.0, 100.0, -150.0, 150.0])
plt.show()


