import sys
sys.path.append('../Planning')

import pygame
from pygame.locals import *
import matplotlib
#matplotlib.use('PS')
import matplotlib.pyplot as plt
import RRT
import numpy

numberOfNodes = 10000
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
    #pygame.draw.line(screen, color, startNode.config, startNode.parent.config)
    #pygame.display.update()
    plt.plot([startNode.config.item(0), startNode.parent.config.item(0)], \
             [startNode.config.item(1), startNode.parent.config.item(1)], \
             color=color, linestyle='-', linewidth=2)
    plt.draw()

  for child in startNode.children:
    VisualizeTree(child, color)

terrain = numpy.ones(500)*(400) 
terrain[100:220] = numpy.ones(120)*(300) 
terrain[280:320] = numpy.ones(40)*(300) 
plt.fill_between(range(0, 500), terrain, 500, color='none', hatch="/", edgecolor='r')
plt.draw()

q_limits = numpy.array([[0.0, 0.0],[500.0, 500.0]])

tree = RRT.Build_RRT(numpy.array([250.0, 250.0]), numberOfNodes, delta_q, terrain, q_limits)
VisualizeTree(tree.root, 'b')

plt.gca().set_xticks(numpy.arange(0,500.,50))
plt.gca().set_yticks(numpy.arange(0,500.,50))
plt.gca().invert_yaxis()
plt.gca().set_aspect('equal')
#plt.gca().set_frame_on(False)
plt.gca().set_xticks([]) 
plt.gca().set_yticks([]) 
#plt.grid()
plt.ylabel('parameter 2')
plt.xlabel('parameter 1')
plt.show()

#plt.savefig('OneTree_Terrain')
