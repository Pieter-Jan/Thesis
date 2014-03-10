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
    plt.plot([startNode.config.item(0), startNode.parent.config.item(0)], \
             [startNode.config.item(1), startNode.parent.config.item(1)], \
             color=color, linestyle='-', linewidth=2)
    plt.draw()

  for child in startNode.children:
    VisualizeTree(child, color)

x_limits = numpy.array([[-50.0, -50.0], [50.0, 50.0]])
x_ideal = numpy.array([0., 13.2436, -23.0818])

P1 = numpy.array([5.0, 5.0])
P2 = numpy.array([5.0, 40.0]) 
P3 = numpy.array([40.0, 5.0]) 

triangleGoal = [P1, P2, P3]

q_init = numpy.array([0.4829, 1.6494, 1.5801, 0.7607, 1.9062, 1.6214, 0.6808,
  1.4351, 1.6197, 0.4765, 1.3898, 1.6237])
x_init = numpy.array([0.0, 0.0, 0.0])
#OK.Relative_COB(q_ref, q_init, swingLeg)
print x_init

tree = TS_RRT.RRT_TriangleGoal_Planner(x_init, q_init, 200, 10.0, triangleGoal,
    x_ideal, x_limits)

# Visualize
ax.add_patch(Polygon([P1, P2, P3], closed=True, alpha=0.5))
ax.plot(x_ideal[1], x_ideal[2], 'ro')
ax.plot(x_init[0], x_init[1], 'bo')
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
