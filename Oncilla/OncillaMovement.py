import sys
sys.path.append('../Planning')

import numpy
import math
import OncillaKinematics as OK
import OncillaVisualization as OV
import RRT 
import Oncilla2
import time
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
numpy.set_printoptions(precision=5)
numpy.set_printoptions(suppress=True)

def QuadraticBezier(P0, P1, P2, t):
  #return (1-2)**2*P0+2*(1-t)*t*P1+t**2*P2
  return P0*(1-t)**2 + 2*P1*(1-t)*t + P2*t**2

def SameSide(p1,p2, a,b):
  cp1 = numpy.cross(b-a, p1-a)
  cp2 = numpy.cross(b-a, p2-a)
  if numpy.dot(cp1, cp2) >= 0: 
    return True
  else: 
    return False

def PointInTriangle(p, a, b, c):
  if SameSide(p,a, b,c) and SameSide(p,b, a,c) and SameSide(p,c, a,b): 
      return True
  else:
    return False

def VectorIntersection(v1,v2,d1,d2):
  # v1 and v2 - Vector points
  # d1 and d2 - Direction vectors
  # returns the intersection point for the two vector line equations.
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
          lmbda = float(v1[0]*d2[1] - v1[1]*d2[0] - v2[0]*d2[1] + 
                        v2[1]*d2[0])/(d1[1]*d2[0] - d1[0]*d2[1])
      elif d2[0] == 0 and d1[0] != 0:
          lmbda = float(v2[0] - v1[0])/d1[0]
      elif d2[1] == 0 and d1[1] != 0:
          lmbda = float(v2[1] - v1[1])/d1[1]
      else:
          raise ValueError('Direction vectors are invalid.')
      return (v1[0] + lmbda* d1[0],v1[1] + lmbda * d1[1])

def MaximumReachPoint(q_current, leg):
  feet = OK.RelativeFootPositions(q_current) 
  supportFeet = numpy.delete(feet, leg-1, axis=1)

  # Determine plane through support feet
  v1 = numpy.squeeze(numpy.asarray(supportFeet[:,1] - supportFeet[:,0]))
  v2 = numpy.squeeze(numpy.asarray(supportFeet[:,2] - supportFeet[:,0]))
  shoulder = numpy.array([[OK.Qs[leg-1]], [OK.Rs[leg-1]], [OK.Ss[leg-1]]])
  v3 = numpy.squeeze(numpy.asarray(shoulder - supportFeet[:,0]))
  
  n = numpy.cross(v1, v2)
  n = n/numpy.linalg.norm(n)

  x = numpy.linalg.norm(numpy.dot(v3, n)*n)
  z = feet[2, leg-1] - shoulder[2]

  max_y = math.sqrt(160.0**2 - x**2 - z**2)  

  X_MR = numpy.array([[x], [max_y], [z]]) + shoulder

  return X_MR 

def MoveToConfiguration(oncilla, q_start, q_goal):
  tree_a, tree_b, plan = RRT.RRT_Connect_Planner(q_start, q_goal, 1000, 0.01, None, OK.q_limits) 
  
  for q in plan:
    reply = oncilla.sendConfiguration(q)

  return q

def MoveLeg(oncilla, q_start, q_goal_leg, leg):
  q_goal = list(q_start)
  q_goal[3*(leg-1):3*(leg-1)+3] = numpy.squeeze(q_goal_leg)
  
  MaximumReachPoint(q_start, 1)

  q = MoveToConfiguration(oncilla, q_start, q_goal)
  
  return q

def SwingLeg(oncilla, q_current, leg):
  X_start = OK.RelativeFootPositions(q_current)

  X_MR = MaximumReachPoint(q_current, leg)
  
  X_control = numpy.matrix([[X_start[0, leg-1]-60.0, X_start[1, leg-1]+(X_MR[1]-X_start[1, leg-1])/2.0, X_MR[2]]]).T

  t = 0.0
  step = 0.05
  while t < 1.0:
    X_next = numpy.matrix(QuadraticBezier(X_start[:,leg-1], X_control, X_MR, t))
    
    t = t + step
    
    q_goal = OK.InverseKinematics_SL(q_current[3*(leg-1):3*(leg-1)+3], X_next, leg)
    q_current[3*(leg-1):3*(leg-1)+3] = q_goal
    reply = oncilla.sendConfiguration(q_current)

    #q_current = MoveLeg(oncilla, q_current, q_goal, leg)
  
  SwingShift(oncilla, q_current, leg)

  return q_current

def Move_COB(oncilla, X_goal, q_init, q_ref, speed, swingLeg=1):
  # Move in a straigth line to X_goal
  # X_goal: Goal position relative to current position
  # q_init: current configuration of robot
  # q_ref: reference configuration (X_B = 0, 0, 0)

  accuracy = 1.0

  X_current = OK.Relative_COB(q_init, q_ref, swingLeg)

  q_current = q_init

  startTime = time.time()

  while numpy.linalg.norm(X_goal - X_current) > accuracy:
    X_next = X_goal - X_current
    norm = numpy.linalg.norm(X_next) 
    X_next = X_next/norm*min(norm, speed*(time.time() - startTime))
    
    startTime = time.time()

    q_current = OK.InverseKinematics_COB_SL(q_current, X_next)
    #q_current = OK.InverseKinematics_COB(q_current, X_next)

    if q_current is not None:
      reply = oncilla.sendConfiguration(q_current)
    else:
      print 'Fail'
      return None

    X_current = OK.Relative_COB(q_ref, q_current, swingLeg) 

  return q_current

def QuadShift(oncilla, q_current, swingLeg):
  # Move the body to position the COG inside the upcoming support polygon
  stabilityMargin = 10.0

  feet = OK.RelativeFootPositions(q_current) 
  feet = numpy.delete(feet, 0, axis=0) # remove z-coordinates

  if swingLeg == 1 or swingLeg == 4:
    P1 = numpy.squeeze(numpy.asarray(feet[:, 1]))
    P2 = numpy.squeeze(numpy.asarray(feet[:, 2]))

  elif swingLeg == 2 or swingLeg == 3:
    P1 = numpy.squeeze(numpy.asarray(feet[:, 0]))
    P2 = numpy.squeeze(numpy.asarray(feet[:, 3]))

  trot_dir = (P1-P2)/numpy.linalg.norm(P1-P2)
 
  # Movement should be ortoghonal to trot line 
  move_dir = numpy.array([trot_dir[1], -trot_dir[0]])

  X_start = numpy.array([0.0, 0.0])
  feet = numpy.delete(feet, swingLeg-1, axis=1)
  if not PointInTriangle(X_start, feet[:, 0].T, feet[:, 1].T, feet[:, 2].T):

    intersection = VectorIntersection(P1, X_start, trot_dir, move_dir)

    X_goal_2D = intersection# - stabilityMargin*move_dir
    X_goal_3D = numpy.matrix([[0.0], [X_goal_2D[0]], [X_goal_2D[1]]])

    q_goal = OK.InverseKinematics_COB_SL(q_current, X_goal_3D)
  
    q_current = MoveToConfiguration(oncilla, q_current, q_goal)

  return q_current

def SwingShift(oncilla, q_current, swingLeg):
  if leg == 3 or leg == 4:
    None
  elif leg == 1 or leg == 2:
    None
    
