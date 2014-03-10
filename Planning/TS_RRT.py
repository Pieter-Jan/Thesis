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
# Warning: This code is very very specific
def euclidian_distance(node1, node2):
  x1 = numpy.delete(node1.state, 0)
  x2 = numpy.delete(node2.state, 0)
  return numpy.linalg.norm(x1 - x2)

class TS_Tree:
  def __init__(self, x_init, q_init):
    self.root = TS_Node(x_init, q_init, self)
    self.mindist = float("inf")
    self.near = None
    self.lastNode = None

  def nearestNode(self, startNode, node):
    dist = euclidian_distance(startNode, node)

    if dist < self.mindist:
      self.mindist = dist
      self.near = startNode

    for child in startNode.children:
      self.nearestNode(child, node)
    
    if startNode.parent == None:
      self.mindist = float("inf")

    return self.near
  
class TS_Node:
  def __init__(self, x, q, tree=None):
    # x: Task space
    # q: Configuration space

    self.children = []
    self.parent = None

    self.config = q
    self.state = x

    self.tree = tree

  def addChild(self, child):
    child.parent = self
    child.tree = self.tree
    self.children.append(child)

    if self.tree is not None:
      self.tree.lastNode = self

def TS_Control(q_near, x_near, x_rand):
  # TODO
  None

def TS_Extend(tree, x_rand, delta_x, terrain):
  randNode = TS_Node(x_rand, None)
  near = tree.nearestNode(tree.root, randNode)
  
  u = randNode.state - near.state 
  u = numpy.delete(u, 0, axis=0)
  u = u/numpy.linalg.norm(u)
  
  nearState_2D = numpy.delete(near.state, 0, axis=0)

  x_new_2D = nearState_2D + u*delta_x
  x_new = numpy.array([near.state[0], x_new_2D[0], x_new_2D[1]])
  swingLeg = 1
  q_new = OK.InverseKinematics_COB_SL(near.config, x_new, swingLeg)

  if q_new is not None:
    global x_new_global   
    x_new_global = x_new

    newNode = TS_Node(x_new, q_new) 
    if RRT.CollisionFree(newNode, terrain):
      near.addChild(newNode)
      if numpy.linalg.norm(x_new - x_rand) < delta_x:
        return RRT.REACHED
      else:
        return RRT.ADVANCED
    else:
      return RRT.TRAPPED

  else: 
    return RRT.TRAPPED

# RRT with triangle goal and "ideal" point
# ATTENTION: This is a very specific implementation for 2D reduction!
def RRT_TriangleGoal_Planner(x_init, q_init, numberOfNodes, delta_x, triangleGoal,
    x_ideal, x_limits):

  tree = TS_Tree(x_init, q_init)
  P1 = triangleGoal[0]
  P2 = triangleGoal[1]
  P3 = triangleGoal[2]

  for k in range(1, numberOfNodes):

    # if numpy.random.randn(0, 1) > 0.7:
    x_rand_2D = x_limits[0,:] + (x_limits[1,:] - x_limits[0,:])*numpy.random.rand(len(x_limits))
    x_rand = numpy.array([x_init[0], x_rand_2D[0], x_rand_2D[1]])
    # else:
    #   x_rand = x_ideal

    print x_rand
    progress = TS_Extend(tree, x_rand, delta_x, None)

    if progress is not RRT.TRAPPED:
      x_new_global_2D = numpy.delete(x_new_global, 0)
      #if MathExtra.PointInTriangle(x_new_global_2D, P1, P2, P3):
      #  break

  return tree


  

    
    
    


