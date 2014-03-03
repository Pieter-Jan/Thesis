# Oncilla kinematics using the Denavit Hartenberg convention
import numpy
import math
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

# Leg parameters
As = [60.0, 60.0, 78.0, 78.0]
Bs = [56.0, 56.0, 65.0, 65.0]
Cs = [62.5, 62.0, 52.0, 52.0]

# Position of legs within robot (hip coordinates)
Qs = [0.0, 0.0, 0.0, 0.0]
Rs = [227.0/2.0, 227.0/2.0, -227.0/2.0, -227.0/2.0]
Ss = [-137.5/2.0, 137.5/2.0, -137.5/2.0, 137.5/2.0]

# Configuration Joint Angle limits
q_limits = math.pi/180.0*numpy.array([[-43., 85., 81.02, 
                                       -43., 85., 81.02, 
                                       -43., 78., 83.62, 
                                       -43., 78., 83.62],
                                      [62., 135., 98.08, 
                                       62., 135., 98.08, 
                                       58., 135., 100.40,  
                                       58., 135., 100.40]])

# Denavit Hartenberg Transformation Matrix
def DH_BODY(Q, R, S, flip):
  if flip:
    s = -1.0 # Flip z-axis
  else:
    s = 1.0

  DH_BODY = numpy.matrix([[1.0, 0.0, 0.0, Q], 
                          [0.0, 1.0, 0.0, R], 
                          [0.0, 0.0, 1.0, S], 
                          [0.0, 0.0, 0.0, 1.0]])

  DH_BODY2 = numpy.matrix([[1.0, 0.0, 0.0, 0.0], 
                           [0.0, 1.0, 0.0, 0.0], 
                           [0.0, 0.0, s, 0.0], 
                           [0.0, 0.0, 0.0, 1.0]])
  return DH_BODY*DH_BODY2

def DH_SERVO(gamma):
  angle = gamma - math.pi/2.0
  DH_SERVO = numpy.matrix([[numpy.cos(angle), 0.0, numpy.sin(angle), 0.0], 
                         [0.0, 1.0, 0.0, 0.0], 
                         [-numpy.sin(angle), 0.0, numpy.cos(angle), 0.0], 
                         [0.0, 0.0, 0.0, 1.0]])
  return DH_SERVO

def DH_HIP(alpha, A):
  DH_HIP = numpy.matrix([[math.cos(alpha), -math.sin(alpha), 0.0, A*math.cos(alpha)], 
                         [math.sin(alpha), math.cos(alpha), 0.0, A*math.sin(alpha)], 
                         [0.0, 0.0, 1.0, 0.0], 
                         [0.0, 0.0, 0.0, 1.0]])
  return DH_HIP

def DH_KNEE(beta, B):
  DH_KNEE = numpy.matrix([[math.cos(beta+math.pi), -math.sin(beta+math.pi), 0.0, B*math.cos(beta+math.pi)], 
                          [math.sin(beta+math.pi), math.cos(beta+math.pi), 0.0, B*math.sin(beta+math.pi)],
                          [0.0, 0.0, 1.0, 0.0], 
                          [0.0, 0.0, 0.0, 1.0]])
  return DH_KNEE

def DH_ANKLE(beta, C):
  DH_ANKLE = numpy.matrix([[math.cos(-beta-math.pi), -math.sin(-beta-math.pi), 0.0, math.cos(-beta-math.pi)*C], 
                           [math.sin(-beta-math.pi), math.cos(-beta-math.pi), 0.0, math.sin(-beta-math.pi)*C], 
                           [0.0, 0.0, 1.0, 0.0], 
                           [0.0, 0.0, 0.0, 1.0]])
  return DH_ANKLE

def Jacobian_SL(alpha, beta, gamma, leg):

  if leg == 1 or leg == 3:
    flip = False

  else:
    flip = True

  A, B, C, Q, R, S = As[leg-1], Bs[leg-1], Cs[leg-1], Qs[leg-1], Rs[leg-1], Ss[leg-1]

  if not flip:
    J = numpy.matrix([[(-(A+C)*math.sin(alpha)+B*math.sin(alpha+beta))*math.sin(gamma), B*math.sin(alpha+beta)*math.sin(gamma), ((A+C)*math.cos(alpha)-B*math.cos(alpha+beta))*math.cos(gamma)], 
                      [(A+C)*math.cos(alpha)-B*math.cos(alpha+beta), -B*math.cos(alpha+beta), 0.0],
                      [(-(A+C)*math.sin(alpha)+B*math.sin(alpha+beta))*math.cos(gamma), B*math.sin(alpha+beta)*math.cos(gamma), (-(A+C)*math.cos(alpha)+B*math.cos(alpha+beta))*math.sin(gamma)]]) 

  else:
    J = numpy.matrix([[(-(A+C)*math.sin(alpha)+B*math.sin(alpha+beta))*math.sin(gamma), B*math.sin(alpha+beta)*math.sin(gamma), ((A+C)*math.cos(alpha)-B*math.cos(alpha+beta))*math.cos(gamma)], 
                      [(A+C)*math.cos(alpha)-B*math.cos(alpha+beta), -B*math.cos(alpha+beta), 0.0],
                      [((A+C)*math.sin(alpha)-B*math.sin(alpha+beta))*math.cos(gamma), -B*math.sin(alpha+beta)*math.cos(gamma), ((A+C)*math.cos(alpha)-B*math.cos(alpha+beta))*math.sin(gamma)]]) 

  return J

def FootPositions_SL(alpha, beta, gamma, leg):
  # Calculates the foot position of leg i: P_L_i

  if leg == 1 or leg == 3:
    flip = False

  else:
    flip = True

  A, B, C, Q, R, S = As[leg-1], Bs[leg-1], Cs[leg-1], Qs[leg-1], Rs[leg-1], Ss[leg-1]

  P_F_i = DH_BODY(Q, R, S, flip)*DH_SERVO(gamma)*DH_HIP(alpha, A)*DH_KNEE(beta, B)*DH_ANKLE(beta, C)*numpy.matrix('0.0; 0.0; 0.0; 1.0') 

  return P_F_i[0:3]

def KneePositions_SL(alpha, beta, gamma, leg):
  # Calculates the knee position of leg i: P_L_i

  if leg == 1 or leg == 3:
    flip = False

  else:
    flip = True

  A, B, C, Q, R, S = As[leg-1], Bs[leg-1], Cs[leg-1], Qs[leg-1], Rs[leg-1], Ss[leg-1]

  P_K_i = DH_BODY(Q, R, S, flip)*DH_SERVO(gamma)*DH_HIP(alpha, A)*numpy.matrix('0.0; 0.0; 0.0; 1.0') 

  return P_K_i[0:3]

def RelativeFootPositions(q_a):
  P_K_1 = FootPositions_SL(q_a[0], q_a[1], q_a[2], 1)
  P_K_2 = FootPositions_SL(q_a[3], q_a[4], q_a[5], 2)
  P_K_3 = FootPositions_SL(q_a[6], q_a[7], q_a[8], 3)
  P_K_4 = FootPositions_SL(q_a[9], q_a[10], q_a[11], 4)
  
  P_F = numpy.hstack([P_K_1, P_K_2, P_K_3, P_K_4])

  return P_F

def RelativeKneePositions(q_a):
  P_K_1 = KneePositions_SL(q_a[0], q_a[1], q_a[2], 1)
  P_K_2 = KneePositions_SL(q_a[3], q_a[4], q_a[5], 2)
  P_K_3 = KneePositions_SL(q_a[6], q_a[7], q_a[8], 3)
  P_K_4 = KneePositions_SL(q_a[9], q_a[10], q_a[11], 4)
  
  P_K = numpy.hstack([P_K_1, P_K_2, P_K_3, P_K_4])

  return P_K

def Jacobian_LegLengths(q_a):
  J_leg1 = Jacobian_SL(q_a[0], q_a[1], q_a[2], 1)    
  J_leg2 = Jacobian_SL(q_a[3], q_a[4], q_a[5], 2)    
  J_leg3 = Jacobian_SL(q_a[6], q_a[7], q_a[8], 3)    
  J_leg4 = Jacobian_SL(q_a[9], q_a[10], q_a[11], 4)  
 
  dPl_on_dqa = numpy.vstack([numpy.hstack([J_leg1, numpy.zeros(shape=(3,9))]),
                             numpy.hstack([numpy.zeros(shape=(3,3)), J_leg2, numpy.zeros(shape=(3,6))]),
                             numpy.hstack([numpy.zeros(shape=(3,6)), J_leg3, numpy.zeros(shape=(3,3))]),
                             numpy.hstack([numpy.zeros(shape=(3,9)), J_leg4])])
  return dPl_on_dqa

def Jacobian_COB(X_B, q_a, swingLeg):
  # X_B is the center of body in the global system
  X_B_4 = numpy.hstack([X_B, X_B, X_B, X_B]) 

  P_L = RelativeFootPositions(q_a)

  R_B = GetRotationMatrix(q_a, swingLeg)

  # Position of 4 feet in global system
  P_G = X_B_4 + R_B*P_L

  # Lengths of the 4 feet
  L = numpy.vstack([numpy.linalg.norm(P_L[:,0]), numpy.linalg.norm(P_L[:,1]), numpy.linalg.norm(P_L[:,2]), numpy.linalg.norm(P_L[:,3])])
  
  dL_on_dXb = numpy.matrix(X_B_4 - P_G).T/L 

  dL_on_dPl = numpy.vstack([numpy.matrix(numpy.hstack([P_L[:,0].T/L[0], numpy.zeros(shape=(1,9))])),
                            numpy.matrix(numpy.hstack([numpy.zeros(shape=(1,3)), P_L[:,1].T/L[1], numpy.zeros(shape=(1,6))])),
                            numpy.matrix(numpy.hstack([numpy.zeros(shape=(1,6)), P_L[:,2].T/L[2], numpy.zeros(shape=(1,3))])),
                            numpy.matrix(numpy.hstack([numpy.zeros(shape=(1,9)), P_L[:,3].T/L[3]]))])

  dPl_on_dqa = Jacobian_LegLengths(q_a)

  J_Xb = numpy.linalg.pinv(dL_on_dXb)*dL_on_dPl*dPl_on_dqa

  return J_Xb

def Jacobian_RBj(X_B, q_a, j):
  # Jacobian of body rotation
  P_L = RelativeFootPositions(q_a)

  R_B = RotationMatrix(P_L[:,1].T, P_L[:,3].T, P_L[:,2].T)

  dPl_on_dqa = Jacobian_LegLengths(q_a)

  dXB_on_dqaj = Jacobian_COB(X_B, q_a)[:, j]

  dXB4_on_dqaj = numpy.hstack([dXB_on_dqaj, dXB_on_dqaj, dXB_on_dqaj, dXB_on_dqaj])

  dPl_on_dqaj = numpy.hstack([dPl_on_dqa[0:3,j], dPl_on_dqa[3:6,j], dPl_on_dqa[6:9,j], dPl_on_dqa[9:12,j]])

  J_RBj = -(dXB4_on_dqaj + dPl_on_dqaj)*numpy.linalg.pinv(P_L)
  
  return J_RBj

def AngleLimits_SL(q_SL, leg):
  # q_SL: 1*3 matrix with alpha, beta, gamma of 1 leg
  for joint in xrange(0, 3):
    q_SL[joint] = max(q_SL[joint], q_limits[0, 3*(leg-1) + joint])
    q_SL[joint] = min(q_SL[joint], q_limits[1, 3*(leg-1) + joint])

  return q_SL

def AngleLimits(q_a):
  # q_a: 1*12 matrix with alpha, beta & gamma of all legs
  for leg in xrange(1,5):
    q_a[3*(leg-1):3*(leg-1)+3] = AngleLimits_SL(q_a[3*(leg-1):3*(leg-1)+3], leg)
  return q_a

def Jacobian_SwingFoot(X_B, q_a, leg=1):
  # Swing Foot Jacobian
  P_L = RelativeFootPositions(q_a)

  R_B = GetRotationMatrix(q_a, leg)
  
  Xsw_L = P_L[:, leg-1]

  dPl_on_dqa = Jacobian_LegLengths(q_a)

  for j in xrange(0, 12):
    if j != 3*(leg-1) and j != 3*(leg-1)+1 and j != 3*(leg-1)+2:
      dXB_on_dqaj = Jacobian_COB(X_B, q_a)[:, j]

      J_RBj = Jacobian_RBj(X_B, q_a, j)

    else:
      dXB_on_dqaj = 0
      
      J_RBj = numpy.matrix(numpy.zeros(shape=(3,3)))

    dXswL_on_dqaj = dPl_on_dqa[(leg-1)*3:(leg-1)*3+3,j] 

    J_Xsw_Gj = dXB_on_dqaj + R_B*dXswL_on_dqaj + J_RBj*Xsw_L
    
    if j == 0:
      J_Xsw_G = J_Xsw_Gj
    else:
      J_Xsw_G = numpy.hstack([J_Xsw_G, J_Xsw_Gj])

  return J_Xsw_G

def RotationMatrix(P1, P2, P3):
  # Calculates the rotation matrix from three points using the rodrigues formula

  n1 = numpy.matrix([1.0,0.0,0.0])

  v1 = P1 - P2
  v2 = P3 - P2

  n2 = numpy.cross(v2, v1)

  if n2.any():
    n2 = n2/numpy.linalg.norm(n2)

  return RodriguesRotation(n2, n1)

def GetRotationMatrix(q_a, leg):
  P_L = RelativeFootPositions(q_a)

  if leg == 1:
    R_B = RotationMatrix(P_L[:,1].T, P_L[:,3].T, P_L[:,2].T)
  elif leg == 2:
    R_B = RotationMatrix(P_L[:,3].T, P_L[:,2].T, P_L[:,0].T)
  elif leg == 3:
    R_B = RotationMatrix(P_L[:,0].T, P_L[:,1].T, P_L[:,3].T)
  elif leg == 4:
    R_B = RotationMatrix(P_L[:,2].T, P_L[:,0].T, P_L[:,1].T)

  return R_B

def RodriguesRotation(n1, n2):

  if n2.any():
    theta = numpy.arccos(numpy.inner(n1, n2)) 
    theta = theta[0,0]
  else:
    theta = 0.0

  x = numpy.cross(n1, n2)
  if x.any():
    x = x/numpy.linalg.norm(x)

  A = numpy.matrix([[0.0, -x[0,2], x[0,1]],
                    [x[0,2], 0.0, -x[0,0]], 
                    [-x[0,1], x[0,0], 0.0]])

  R = numpy.identity(3) + numpy.sin(theta)*A + (1.0-numpy.cos(theta))*A**2
  
  return R

def RotationMatrix_PRY(yaw, pitch,roll):
  alpha = yaw
  beta = pitch
  gamma = roll
  R = numpy.matrix([[math.cos(alpha)*math.cos(beta), math.cos(alpha)*math.sin(beta)*math.sin(gamma)-math.sin(alpha)*math.cos(gamma), math.cos(alpha)*math.sin(beta)*math.cos(gamma)+math.sin(alpha)*math.sin(gamma)],
                    [math.sin(alpha)*math.cos(beta), math.sin(alpha)*math.sin(beta)*math.sin(gamma)+math.cos(alpha)*math.cos(gamma), math.sin(alpha)*math.sin(beta)*math.cos(gamma)-math.cos(alpha)*math.sin(gamma)],
                    [-math.sin(beta), math.cos(beta)*math.sin(gamma), math.cos(beta)*math.cos(gamma)]])

  return R

def Trilateration(P1, P2, P3, A, B, C):

  G = numpy.matrix([[P1[0,0], P1[1,0], P1[2,0]],
                    [P2[0,0], P2[1,0], P2[2,0]], 
                    [P3[0,0], P3[1,0], P3[2,0]]]) 

  h = 1.0/2.0*numpy.matrix([[P1[0,0]**2 + P1[1,0]**2 + P1[2,0]**2 - A**2],
                        [P2[0,0]**2 + P2[1,0]**2 + P2[2,0]**2 - B**2],
                        [P3[0,0]**2 + P3[1,0]**2 + P3[2,0]**2 - C**2]])

  return numpy.linalg.pinv(G)*h

def InverseKinematics_SL(q_init_leg, X_goal_leg, leg):
  accuracy = 0.01
  maxIter = 100

  X_current = FootPositions_SL(q_init_leg[0], q_init_leg[1], q_init_leg[2], leg)
  
  q_cur = q_init_leg

  i = 0
  distance = numpy.linalg.norm(X_goal_leg - X_current)
  while distance > accuracy and i < maxIter:
    J = Jacobian_SL(q_cur[0], q_cur[1], q_cur[2], leg)

    dq = numpy.linalg.pinv(J)*(X_goal_leg - X_current)

    q_cur = numpy.squeeze(numpy.asarray(q_cur + dq.T))
    q_cur = AngleLimits_SL(q_cur, leg)
  
    X_current = FootPositions_SL(q_cur[0], q_cur[1], q_cur[2], leg) 

    distance = numpy.linalg.norm(X_goal_leg - X_current)

    i = i + 1

  if i <= maxIter:
    return q_cur
  else:
    print 'Fail'
    return None

def InverseKinematics_COB(q_init, X_G):
  P_start = RelativeFootPositions(q_init)
  X_B_Current = numpy.matrix([[0.0], [0.0], [0.0]])

  maxIter = 100
  accuracy = 0.05

  i = 0
  qa = q_init
  while numpy.linalg.norm(X_G - X_B_Current) > accuracy and i < maxIter:
    J = Jacobian_COB(X_B_Current, qa);

    dq = numpy.squeeze(numpy.asarray(numpy.linalg.pinv(J)*(X_G - X_B_Current)))

    qa = qa + dq.T
  
    qa = AngleLimits(qa)
    
    X_B_Current = Relative_COB(q_init, qa, 1) 
  
    i = i + 1

  if i < maxIter:
    return qa
  else:
    print 'Fail'
    return None

def InverseKinematics_COB_SL(q_init, X_G, swingLeg):
  # Inverse kinematics for COB based on single leg jacobians
  # gives configuration q to move to goal state X_G relative to current state
  
  X_B_Current = numpy.matrix([[0.0, 0.0, 0.0]])

  maxIter = 100
  accuracy = 1.0

  qa = q_init 

  i = 0
  while numpy.linalg.norm(X_G - X_B_Current) > accuracy and i < maxIter:
    X_dot = X_G - X_B_Current
  
    R_B = GetRotationMatrix(qa, swingLeg)

    P_L_dot = R_B.T*(-X_dot)
  
    J_leg1 = Jacobian_SL(qa[0], qa[1], qa[2], 1)    
    J_leg2 = Jacobian_SL(qa[3], qa[4], qa[5], 2)    
    J_leg3 = Jacobian_SL(qa[6], qa[7], qa[8], 3)    
    J_leg4 = Jacobian_SL(qa[9], qa[10], qa[11], 4)  

    qa1 = numpy.linalg.pinv(J_leg1)*P_L_dot
    qa2 = numpy.linalg.pinv(J_leg2)*P_L_dot
    qa3 = numpy.linalg.pinv(J_leg3)*P_L_dot
    qa4 = numpy.linalg.pinv(J_leg4)*P_L_dot

    qa = qa + numpy.hstack([qa1.T.tolist()[0], 
                            qa2.T.tolist()[0],  
                            qa3.T.tolist()[0], 
                            qa4.T.tolist()[0]])

    qa = AngleLimits(qa)

    X_B_Current = Relative_COB(q_init, qa, 1) 

    i += 1
     
  if i >= maxIter:
    print 'InverseKinematics_COB_SL failed'

  return qa

def Relative_COB(q_start, q_current, swingLeg=1):
  # Calculates the position 
  # swingLeg: leg that is not on the ground

  P_start = RelativeFootPositions(q_start)
  P_start = numpy.delete(P_start, swingLeg-1, axis=1)

  P_current = RelativeFootPositions(q_current)
  P_current = numpy.delete(P_current, swingLeg-1, axis=1)
  
  A = numpy.linalg.norm(P_current[:,0])
  B = numpy.linalg.norm(P_current[:,1])
  C = numpy.linalg.norm(P_current[:,2])

  return Trilateration(P_start[:,0], P_start[:,1], P_start[:,2], A, B, C)

