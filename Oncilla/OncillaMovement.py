import numpy
import math
import OncillaKinematics as OK
import OncillaVisualization as OV
import Oncilla2
import time
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
numpy.set_printoptions(precision=5)
numpy.set_printoptions(suppress=True)

def vector_intersection(v1,v2,d1,d2):
    '''
    v1 and v2 - Vector points
    d1 and d2 - Direction vectors
    returns the intersection point for the two vector line equations.
    '''
    if d1[0] == 0 and d2[0] != 0 or d1[1] == 0 and d2[1] != 0:
        if d1[0] == 0 and d2[0] != 0:
            mu = float(v1[0] - v2[0])/d2[0]
        elif d1[1] == 0 and d2[1] != 0:
            mu = float(v1[1] - v2[1])/d2[1]
        return (v2[0] + mu* d2[0],v2[1] + mu * d2[1])
    else:
        if d1[0] != 0 and d1[1] != 0 and d2[0] != 0 and d2[1] != 0:
            if d1[1]*d2[0] - d1[0]*d2[1] == 0:
                raise ValueError('Direction vectors are invalid. (Parallel)')
            lmbda = float(v1[0]*d2[1] - v1[1]*d2[0] - v2[0]*d2[1] + v2[1]*d2[0])/(d1[1]*d2[0] - d1[0]*d2[1])
        elif d2[0] == 0 and d1[0] != 0:
            lmbda = float(v2[0] - v1[0])/d1[0]
        elif d2[1] == 0 and d1[1] != 0:
            lmbda = float(v2[1] - v1[1])/d1[1]
        else:
            raise ValueError('Direction vectors are invalid.')
        return (v1[0] + lmbda* d1[0],v1[1] + lmbda * d1[1])

def Move_Leg(oncilla):
  # TODO
  None

def Move_COB(oncilla, X_goal, q_init, q_ref, speed):
  # Move in a straigth line to X_goal
  # X_goal: Goal position relative to current position
  # q_init: current configuration of robot
  # q_ref: reference configuration (X_B = 0, 0, 0)

  swingLeg = 1 
  accuracy = 1.0

  X_current = OK.Relative_COB(q_init, q_ref, swingLeg)

  q_current = q_init

  startTime = time.time()

  while numpy.linalg.norm(X_goal - X_current) > accuracy:
    X_next = X_goal - X_current
    norm = numpy.linalg.norm(X_goal - X_current) 
    #if norm > 10.0*accuracy:
    X_next = X_next/norm * (speed*(time.time() - startTime))
    
    startTime = time.time()

    q_current = OK.InverseKinematics_COB_SL(q_current, X_next)
    #q_current = OK.InverseKinematics_COB(q_current, X_next)

    #print q_current * 180.0/math.pi

    if q_current is not None:
      reply = oncilla.sendConfiguration(q_current)
    else:
      print 'Fail'
      return None

    X_current = OK.Relative_COB(q_ref, q_current, swingLeg) 
    #print X_current.T

  return q_current

def QuadShift(q_current, swingLeg):
  # Move the body to position the COG inside the upcoming support polygon
  stabilityMargin = 2.0

  feet = OK.RelativeFootPositions(q_current) 
  feet = numpy.delete(feet, 0, axis=0) # remove z-coordinates
  print feet

  if swingLeg == 1 or swingLeg == 4:
    P1 = numpy.squeeze(numpy.asarray(feet[:, 1]))
    P2 = numpy.squeeze(numpy.asarray(feet[:, 2]))

  elif swingLeg == 2 or swingLeg == 3:
    P1 = numpy.squeeze(numpy.asarray(feet[:, 0]))
    P2 = numpy.squeeze(numpy.asarray(feet[:, 3]))

  trot_dir = (P1-P2)/numpy.linalg.norm(P1-P2)
 
  # Movement should be ortoghonal to trot line 
  move_dir = numpy.array([trot_dir[1], -trot_dir[0]])

  COB_start = numpy.array([0.0, 0.0])
  intersection = vector_intersection(P1, COB_start, trot_dir, move_dir)

  feet = numpy.delete(feet, swingLeg-1, axis=1)
  fig, ax = plt.subplots()
  ax.add_patch(Polygon(feet.T, closed=True, alpha=0.6))
  ax.set_xlim([-200.0, 200.0])
  ax.set_ylim([-100.0, 100.0])
  ax.plot(intersection[0], intersection[1], 'ro')
  ax.plot(0.0, 0.0, 'ko')
  plt.show()
