# Pieter-Jan Van de Maele 
import sys
sys.path.append('../Planning')

import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import math
import numpy
import TS_RRT

fig, ax = plt.subplots()

def VisualizeTree(startNode, color):
  if startNode.parent is not None:
    plt.plot([startNode.config.item(0), startNode.parent.config.item(0)], \
             [startNode.config.item(1), startNode.parent.config.item(1)], \
             color=color, linestyle='-', linewidth=2)
    plt.draw()

  for child in startNode.children:
    VisualizeTree(child, color)

q_init = numpy.array([-90, 30])
ax.plot(q_init[0], q_init[1], 'bo')

q_limits = numpy.array([[-100.0, -100.0], [100.0, 100.0]])

q_ideal = numpy.array([10.0, 10.0])
ax.plot(q_ideal[0], q_ideal[1], 'ro')

P1 = numpy.array([5.0, 5.0])
P2 = numpy.array([5.0, 40.0]) 
P3 = numpy.array([40.0, 5.0]) 

ax.add_patch(Polygon([P1, P2, P3], closed=True, alpha=0.5))

triangleGoal = [P1, P2, P3]

tree = TS_RRT.RRT_TriangleGoal_Planner(q_init, 1000, 1.0, triangleGoal,
    q_ideal, q_limits)

VisualizeTree(tree.root, 'k')

plt.gca().set_xticks(numpy.arange(-100,100.,25))
plt.gca().set_yticks(numpy.arange(-100,100.,25))
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
