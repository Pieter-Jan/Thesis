# Pieter-Jan Van de Maele 
import sys
sys.path.append('../Planning')
sys.path.append('../Global')
sys.path.append('../Oncilla')

import random 
import math
import numpy
import RRT
import MathExtra
import OncillaKinematics as OK

def euclidian_distance(x1, x2):
  return numpy.linalg.norm(x1 - x2)

class TS_Tree:
  def __init__(self, q0):
    x0 = numpy.array([0.0, 0.0, 0.0])
    self.root = TS_Node(x0, q0, self)
    self.mindist = float("inf")
    self.near = None
    self.lastNode = None

  def nearestNode(self, startNode, xRand):
      dist = euclidian_distance(startNode.x, xRand)

      if dist < self.mindist:
          self.mindist = dist
          self.near = startNode
      for child in startNode.children:
          self.nearestNode(child, child.x)
      
      if startNode.parent == None:
          self.mindist = float("inf")

      return self.near
  
class TS_Node:
    def __init__(self, x, q, tree=None):
        self.children = []
        self.parent = None
        self.q = q
        self.x = x
        self.tree = tree

    def addChild(self, child):
        child.parent = self
        child.tree = self.tree
        self.children.append(child)
        if self.tree is not None:
            self.tree.lastNode = self

# -----------------------------------------------------------------------------
def Build_TS_RRT(q0, xGoal, K, dq, triangleGoal, xLimits):
    # q0: initial configuration
    # qGoal: goal configuration
    # K: number of nodes   
    swingLeg = 4

    tree = TS_Tree(q0)

    for k in xrange(1, K):
        qNew = None
        while qNew is None:
            P = random.random() 
            if P <= 0.8:
                pRand = numpy.asarray([random.random() for _ in range(0,
                    len(xGoal))])
                xRand = xLimits[0,:] + pRand*(xLimits[1,:] - xLimits[0, :])
            else:
                xRand = xGoal

            nearNode = tree.nearestNode(tree.root, xRand)
            u = (xRand - nearNode.x)/numpy.linalg.norm(xRand - nearNode.x)

            xNew = xLimits[0,:] + u*dq*(xLimits[1,:] - xLimits[0, :])
            xNew[0] = 0.0
            qNew = OK.InverseKinematics_COB_SL(nearNode.q, xNew, swingLeg)

        print 'Node added'
        newNode = TS_Node(xNew, qNew)
        nearNode.addChild(newNode)

    return tree

