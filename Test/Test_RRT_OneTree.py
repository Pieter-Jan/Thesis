import sys
sys.path.append('../Planning')

import pygame
from pygame.locals import *
import matplotlib
#matplotlib.use('PS')
import matplotlib.pyplot as plt
import RRT
import numpy

numberOfNodes = 1000
delta_q = 1.0*10

XDIM = 500
YDIM = 500
WINSIZE = [XDIM, YDIM]
white = 255, 255, 255
red = 255, 0, 0
blue = 0, 0, 255
green = 0, 255, 0
black = 0, 0, 0

def VisualizeTree(startNode, color):
  if startNode.parent is not None:
    plt.plot([startNode.config.item(0), startNode.parent.config.item(0)], \
             [startNode.config.item(1), startNode.parent.config.item(1)], \
             color=color, linestyle='-', linewidth=2)
    plt.draw()

  for child in startNode.children:
    VisualizeTree(child, color)

terrain = numpy.ones(500)*(500) 

q_limits = numpy.array([[0.0, 0.0],[500.0, 500.0]])

tree = RRT.Build_RRT(numpy.array([250.0, 250.0]), numberOfNodes, delta_q, terrain, q_limits)
VisualizeTree(tree.root, 'b')

#plt.gca().set_xticks(numpy.arange(0,500.,50))
#plt.gca().set_yticks(numpy.arange(0,500.,50))
plt.gca().invert_yaxis()
plt.gca().set_aspect('equal')
#plt.gca().set_frame_on(False)
#plt.gca().set_xticks([]) 
#plt.gca().set_yticks([]) 
#plt.grid()
plt.ylabel('parameter 2')
plt.xlabel('parameter 1')
plt.show()

#plt.savefig('OneTree_NoBias_1000')
  
