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

def CenterOfLineSegment(P1, P2):
  x = (P1[0] + P2[0]) / 2.0
  y = (P1[1] + P2[1]) / 2.0
  
  return numpy.array([x, y])

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

def SupportPolygonCentroid(q_current, swingLeg=1):
  feet = OK.RelativeFootPositions(q_current)
  feet = numpy.delete(feet, 0, axis=0)
  feet = numpy.delete(feet, swingLeg-1, axis=1)

  # Centroid of support triangle
  yc = numpy.sum(feet[0,:])*1.0/3.0
  zc = numpy.sum(feet[1,:])*1.0/3.0
  
  return yc, zc

def MaxLegLength(leg):
  if leg == 1 or leg == 2:
    return 150.0
  elif leg == 3 or leg == 4:
    return 165.0

def MaximumReachPoint(q_current, leg):
  maxLegLength = MaxLegLength(leg)

  feet = OK.RelativeFootPositions(q_current) 
  supportFeet = numpy.delete(feet, leg-1, axis=1)

  # Determine plane through support feet
  v1 = numpy.squeeze(numpy.asarray(supportFeet[:,1] - supportFeet[:,0]))
  v2 = numpy.squeeze(numpy.asarray(supportFeet[:,2] - supportFeet[:,0]))
  n = numpy.cross(v1, v2)
  n = n/numpy.linalg.norm(n)

  shoulder = numpy.array([OK.Qs[leg-1], OK.Rs[leg-1], OK.Ss[leg-1]])
  v3 = numpy.squeeze(numpy.asarray(supportFeet[:,0])) - shoulder
  x = numpy.linalg.norm(numpy.dot(v3, n))
  z = feet[2, leg-1] - shoulder[2]
  arg = maxLegLength**2 - x**2 - z**2
  
  if arg > 0:
    y = math.sqrt(arg)
  else:
    y = -1.0*float("inf")

  X_MR = numpy.array([x, y, z]) + shoulder

  return X_MR 

def MoveToConfiguration(oncilla, q_start, q_goal):
  tree_a, tree_b, plan = RRT.RRT_Connect_Planner(q_start, q_goal, 1000, 0.01, 
                                                 None, OK.q_limits) 
  
  if plan is not RRT.FAILURE:
    for q in plan:
      reply = oncilla.sendConfiguration(q)
    return q
  else:
    return q_start

def SwingLeg(oncilla, q_current, leg):
  X_start = OK.RelativeFootPositions(q_current)

  X_MR = MaximumReachPoint(q_current, leg)
  print 'X_MR after shift: \t', X_MR
  X_MR = numpy.matrix(X_MR.reshape(3,1))

  X_control = numpy.matrix([[X_start[0, leg-1]-20.0, 
                             X_start[1, leg-1]+(X_MR[1]-X_start[1, leg-1])/2.0
                             ]]).T
  t = 0.0
  step = 0.01
  while t < 1.0:
    X_next_2D = numpy.squeeze(numpy.asarray(QuadraticBezier(X_start[0:2,leg-1], X_control,
      X_MR[0:2], t)))

    X_next = numpy.matrix([[X_next_2D[0], X_next_2D[1], X_MR[2,0]]]).T
    
    t = t + step
    
    print '\t \t', X_next.T

    q_goal = OK.InverseKinematics_SL(q_current[3*(leg-1):3*(leg-1)+3], 
                                     X_next, leg)
    q_current[3*(leg-1):3*(leg-1)+3] = q_goal
    reply = oncilla.sendConfiguration(q_current)

  return q_current

def MoveCOB(oncilla, X_goal, q_init, q_ref, speed, swingLeg):
  # Move in a straigth line to X_goal
  # X_goal: Goal position relative to current position
  # q_init: current configuration of robot
  # q_ref: reference configuration (X_B = 0, 0, 0)

  q_goal = OK.InverseKinematics_COB_SL(q_init, X_goal, swingLeg)
  q_current = MoveToConfiguration(oncilla, q_init, q_goal)

  #accuracy = 1.0

  #X_current = OK.Relative_COB(q_init, q_ref, swingLeg)

  #q_current = q_init

  #startTime = time.time()

  #while numpy.linalg.norm(X_goal - X_current) > accuracy:
  #  X_next = X_goal - X_current
  #  norm = numpy.linalg.norm(X_next) 
  #  X_next = X_next/norm*min(norm, speed*(time.time() - startTime))
  #  
  #  startTime = time.time()

  #  # q_current = OK.InverseKinematics_COB(q_current, X_next, swingLeg)
  #  q_current = OK.InverseKinematics_COB_SL(q_current, X_next, swingLeg)

  #  if q_current is not None:
  #    reply = oncilla.sendConfiguration(q_current)
  #  else:
  #    print 'Fail'
  #    return None

  #  X_current = OK.Relative_COB(q_ref, q_current, swingLeg) 

  return q_current

def ReducedSupportPolygon(q, margin, swingLeg):
  # Returnes the three points of the support polygon reduced by a certain margin

  feet = OK.RelativeFootPositions(q) 
  feet = numpy.delete(feet, 0, axis=0) # remove x-coordinates

  feet = numpy.delete(feet, swingLeg-1, axis=1)

  P1 = numpy.squeeze(numpy.asarray(feet[:,0]))
  P2 = numpy.squeeze(numpy.asarray(feet[:,1]))
  P3 = numpy.squeeze(numpy.asarray(feet[:,2]))

  u21 = (P2 - P1)/numpy.linalg.norm(P2 - P1)
  u31 = (P3 - P1)/numpy.linalg.norm(P3 - P1)
  u32 = (P3 - P2)/numpy.linalg.norm(P3 - P2)
  
  P1_acc = P1 + u31*margin + u21*margin
  P2_acc = P2 - u21*margin + u32*margin
  P3_acc = P3 - u32*margin - u31*margin

  return P1_acc, P2_acc, P3_acc

def GaitSelection(q_current, u, prevLeg):
  print 'Gait Selection'
  feet = OK.RelativeFootPositions(q_current) 
  legs = [1, 1, 1, 1]
  prevRelProgress = -1.0*float('inf')
  bestLeg = 0

  if prevLeg != 0:
    legs[prevLeg-1] = 0

  #if prevLeg == 1 or prevLeg == 2:
  #  legs[0] = 0
  #  legs[1] = 0
  #elif prevLeg == 3 or prevLeg == 4:
  #  legs[2] = 0
  #  legs[3] = 0

  for leg in xrange(1, 5):
    if legs[leg-1] == 1:
      X_MR = MaximumReachPoint(q_current, leg)
      shoulder = numpy.array([OK.Qs[leg-1], OK.Rs[leg-1], OK.Ss[leg-1]])
      progressVector = X_MR - numpy.squeeze(numpy.asarray(feet[:,leg-1]))

      progress = numpy.dot(progressVector, u)

      relProgress = progress/MaxLegLength(leg)

      print 'leg ', leg
      print '\tprogress: \t', relProgress
      print '\tX_MR: \t ', X_MR

      if relProgress > prevRelProgress:
        prevRelProgress = relProgress
        bestLeg = leg
      
  return bestLeg

def QuadShift(oncilla, q_current, swingLeg):
  # Move the body to position the COG inside the upcoming support polygon
  margin = 5.0

  feet = OK.RelativeFootPositions(q_current) 
  feet = numpy.delete(feet, 0, axis=0) # remove x-coordinates

  P1, P2, P3 = ReducedSupportPolygon(q_current, margin, swingLeg)

  if swingLeg == 2 or swingLeg == 3:
    trot_dir = (P3-P1)/numpy.linalg.norm(P3-P1)
  elif swingLeg == 1 or swingLeg == 4:
    trot_dir = (P3-P2)/numpy.linalg.norm(P3-P2)

  # Movement should be ortoghonal to trot line 
  move_dir = numpy.array([trot_dir[1], -trot_dir[0]])
  if swingLeg == 3 or swingLeg == 4:
    move_dir = -1.0*move_dir

  X_start = numpy.array([0.0, 0.0])
  if not PointInTriangle(X_start, P1, P2, P3):
    X_goal_2D = VectorIntersection(P3, X_start, trot_dir, move_dir)
    X_goal_3D = numpy.matrix([[0.0], [X_goal_2D[0]], [X_goal_2D[1]]])

    q_goal = OK.InverseKinematics_COB_SL(q_current, X_goal_3D, swingLeg)
    q_current = MoveToConfiguration(oncilla, q_current, q_goal)

  return q_current

def CenterOfFrontStabilityLine(q_current, swingLeg):

  feet = OK.RelativeFootPositions(q_current) 
  feet = numpy.delete(feet, 0, axis=0) # remove x-coordinates

  if swingLeg == 1:
    P1 = numpy.squeeze(numpy.asarray(feet[:,2]))
    P2 = numpy.squeeze(numpy.asarray(feet[:,1]))

  elif swingLeg == 2:
    P1 = numpy.squeeze(numpy.asarray(feet[:,0]))
    P2 = numpy.squeeze(numpy.asarray(feet[:,3]))

  elif swingLeg == 3 or swingLeg == 4:
    P1 = numpy.squeeze(numpy.asarray(feet[:,0]))
    P2 = numpy.squeeze(numpy.asarray(feet[:,1]))

  return CenterOfLineSegment(P1, P2)

def SwingShift(oncilla, q_current, leg):
  frontStabilityLineCenter = CenterOfFrontStabilityLine(q_current, leg)

  if leg == 1 or leg == 2:
    X_goal_2D = frontStabilityLineCenter
  elif leg == 3 or leg == 4:
    X_goal_2D = CenterOfLineSegment(frontStabilityLineCenter, 
                                    numpy.array([0.0, 0.0]))

  X_goal_3D = numpy.array([0.0, X_goal_2D[0], X_goal_2D[1]])
  X_goal = numpy.resize(X_goal_3D, (3,1))
  q_goal = OK.InverseKinematics_COB_SL(q_current, X_goal, leg)
  q_current = MoveToConfiguration(oncilla, q_current, q_goal)

  return q_current
  
def StaticGait(oncilla, q_current, u):
  leg = 0     
  q = q_current

  for i in xrange(0,16):
    leg = GaitSelection(q, u, leg)
    print 'Chosen leg: ', leg
    oncilla.swingLeg = leg
    q = QuadShift(oncilla, q, leg)
    q = SwingShift(oncilla, q, leg)
    q = SwingLeg(oncilla, q, leg)
    time.sleep(2)
    print '------------------------------' 

