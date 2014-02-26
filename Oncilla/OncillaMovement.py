import numpy
import math
import OncillaKinematics as OK
import OncillaVisualization as OV
import Oncilla2
import time
#import matplotlib.pyplot as plt
numpy.set_printoptions(precision=5)
numpy.set_printoptions(suppress=True)

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
    print X_current.T

  return q_current

