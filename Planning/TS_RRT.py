# Pieter-Jan Van de Maele 
import sys
sys.path.append('../Planning')
sys.path.append('../Global')
sys.path.append('../Oncilla')

import math
import numpy
import RRT
import MathExtra
import OncillaKinematics as OK

# -----------------------------------------------------------------------------

def TS_Control():
  # TODO
  None

def TS_Extend(tree, q_rand, delta_q, terrain):
  randNode = RRT.Node(q_rand)
  near = tree.nearestNode(tree.root, randNode)
  
  u = randNode.config - near.config
  u = u/numpy.linalg.norm(u)
  
  q_new = near.config + u*delta_q
  global q_new_global   
  q_new_global = q_new

  newNode = RRT.Node(q_new) 
  if RRT.CollisionFree(newNode, terrain):
    near.addChild(newNode)
    if numpy.linalg.norm(q_new - q_rand) < delta_q:
      return RRT.REACHED
    else:
      return RRT.ADVANCED
  else:
    return RRT.TRAPPED

# RRT with triangle goal and "ideal" point
# ATTENTION: This is a very specific implementation for 2D reduction!
def RRT_TriangleGoal_Planner(q_init, numberOfNodes, delta_q, triangleGoal,
    q_ideal, q_limits):

  tree = RRT.Tree(q_init)
  P1 = triangleGoal[0]
  P2 = triangleGoal[1]
  P3 = triangleGoal[2]

  for k in range(1, numberOfNodes):
    if numpy.random.randn() > 0.7:
      q_rand = q_limits[0,:] + (q_limits[1,:] - q_limits[0,:])*numpy.random.rand(len(q_limits))
    else:
      q_rand = q_ideal

    progress = TS_Extend(tree, q_rand, delta_q, None)

    if progress is not RRT.TRAPPED:
      if MathExtra.PointInTriangle(q_new_global, P1, P2, P3):
        break

  return tree


  

    
    
    


