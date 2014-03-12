# Pieter-Jan Van de Maele 
import sys
sys.path.append('../Planning')
sys.path.append('../Oncilla')

import OncillaKinematics as OK
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import math
import numpy
import TS_RRT

q_ref = numpy.array([0.0, 135.0, 90.0, 
                     0.0, 135.0, 90.0, 
                     0.0, 135.0, 90.0, 
                     0.0, 135.0, 90.0])*math.pi/180.0
swingLeg = 1

fig, ax = plt.subplots()

def VisualizeTree(startNode, color):
  if startNode.parent is not None:
    plt.plot([startNode.x.item(1), startNode.parent.x.item(1)], \
             [startNode.x.item(2), startNode.parent.x.item(2)], \
             color=color, linestyle='-', linewidth=2)
    plt.draw()

  for child in startNode.children:
    VisualizeTree(child, color)

xLimits = numpy.array([[0.0, -10.0, -10.0], [10.0, 10.0, 10.0]])
xGoal = numpy.array([0., 13.2436, -23.0818])

P1 = numpy.array([5.0, 5.0])
P2 = numpy.array([5.0, 40.0]) 
P3 = numpy.array([40.0, 5.0]) 
triangleGoal = [P1, P2, P3]

q0 = numpy.array([0.4829, 1.6494, 1.5801, 0.7607, 1.9062, 1.6214, 0.6808,
  1.4351, 1.6197, 0.4765, 1.3898, 1.6237])

tree = TS_RRT.Build_TS_RRT(q0, xGoal, 200, 0.2, triangleGoal, xLimits)

# Visualize
ax.add_patch(Polygon([P1, P2, P3], closed=True, alpha=0.5))
ax.plot(xGoal[1], xGoal[2], 'ro')
VisualizeTree(tree.root, 'k')

plt.gca().set_xticks(numpy.arange(-50,50.,25))
plt.gca().set_yticks(numpy.arange(-50,50.,25))
plt.gca().invert_yaxis()
plt.gca().set_aspect('equal')
# plt.gca().set_frame_on(False)
# plt.gca().set_xticks([]) 
# plt.gca().set_yticks([]) 
# plt.grid()
plt.ylabel('Y')
plt.xlabel('X')
plt.show()

# plt.savefig('OneTree')
