import numpy
import numpy as np
import math
import matplotlib.pyplot as plt
import OncillaKinematics as OK
import pygame
from pygame.locals import *

from mpl_toolkits.mplot3d import Axes3D

white = 255, 255, 255
yellow = 255, 255, 0
green = 0, 255, 0 
blue = 0, 0, 255 
red = 255, 0, 0 
black = 0, 0, 0


def VisualizeRobot(oncilla, q):
  feet = OK.RelativeFootPositions(q) 
  knees = OK.RelativeKneePositions(q) 

  oncilla.screen.fill(black)

  alpha1 = alpha2 = 0.0*math.pi/180.0
  beta1 = beta2 = 135.0*math.pi/180.0
  gamma1 = gamma2 = 90.0*math.pi/180.0
  
  alpha3 = alpha4 = 0.0*math.pi/180.0
  beta3 = beta4 = 135.0*math.pi/180.0
  gamma3 = gamma4 = 90.0*math.pi/180.0
  
  q_start = numpy.array([alpha1, beta1, gamma1, alpha2, beta2, gamma2, alpha3, beta3, gamma3, alpha4, beta4, gamma4])

  x, y, z = OK.Relative_COB(q_start, q, 1)
  
  # XY Plane
  conv = np.asarray([int(250 + y), int(250 + x)])
  for leg in range(0, 4):
    pygame.draw.line(oncilla.screen, white, np.asarray([0.0, 0.0]) + conv, np.asarray([OK.Rs[leg], 0.0]) + conv, 5)
    pygame.draw.line(oncilla.screen, white, np.asarray([OK.Rs[leg], 0.0]) + conv, np.array([knees[1, leg], knees[0, leg]]) + conv, 5)
    pygame.draw.line(oncilla.screen, white, np.array([feet[1, leg], feet[0, leg]]) + conv, np.array([knees[1, leg], knees[0, leg]]) + conv, 5)

    pygame.draw.circle(oncilla.screen, yellow, np.array([int(OK.Rs[leg]), 0]) + conv, 10)
    pygame.draw.circle(oncilla.screen, blue, np.array([int(feet[1, leg]), int(feet[0, leg])]) + conv, 10)
    pygame.draw.circle(oncilla.screen, red, np.array([int(knees[1, leg]), int(knees[0, leg])]) + conv, 10)

  # YZ Plane
  conv = np.asarray([int(750 + y), int(250 + z)])
  for leg in range(0, 4):
    pygame.draw.line(oncilla.screen, white, np.array([feet[1, leg], feet[2, leg]]) + conv, np.array([knees[1, leg], knees[2, leg]]) + conv, 3)
    pygame.draw.circle(oncilla.screen, blue, np.array([int(feet[1, leg]), int(feet[2, leg])]) + conv, 6)

  for leg in range(0, 4):
    pygame.draw.line(oncilla.screen, white, np.array([OK.Rs[leg], OK.Ss[leg]]) + conv, np.array([knees[1, leg], knees[2, leg]]) + conv, 4)
    pygame.draw.circle(oncilla.screen, red, np.array([int(knees[1, leg]), int(knees[2, leg])]) + conv, 8)

  pointlist = [np.asarray([OK.Rs[0], OK.Ss[0]]) + conv, np.asarray([OK.Rs[1], OK.Ss[1]]) + conv, np.asarray([OK.Rs[3], OK.Ss[3]]) + conv, np.asarray([OK.Rs[2], OK.Ss[2]]) + conv] 
  pygame.draw.polygon(oncilla.screen, white, pointlist, 5)
  for leg in range(0, 4):
    pygame.draw.circle(oncilla.screen, yellow, np.array([int(OK.Rs[leg]), int(OK.Ss[leg])]) + conv, 10)





  pygame.display.update()

def VisualizeRobot2(q):
  # 3D

  feet = OK.RelativeFootPositions(q) 
  knees = OK.RelativeKneePositions(q) 

  fig = plt.figure()
  ax = fig.add_subplot(111, projection='3d')
  
  for leg in range(0, 4):
    ax.plot([OK.R[leg], 0], \
            [0, 0], \
            [OK.S[leg], 0], \
            color='b', linestyle='-', linewidth=10)

    ax.plot([OK.R[leg], knees[1, leg]], \
             [0, knees[0, leg]], \
             [OK.S[leg], knees[2, leg]], \
             color='b', linestyle='-', linewidth=10)

    ax.plot([feet[1, leg], knees[1, leg]], \
            [feet[0, leg], knees[0, leg]], \
            [feet[2, leg], knees[2, leg]], \
            color='b', linestyle='-', linewidth=10)

    ax.plot([OK.R[leg]], [0.0], [OK.S[leg]], 'ro', markersize = 20)
    ax.plot([feet[1, leg]], [feet[0, leg]], [feet[2, leg]], 'ro', markersize = 20)
    plt.plot([knees[1, leg]], [knees[0, leg]], [knees[2, leg]], 'ro', markersize = 20)
    plt.axis('equal')

