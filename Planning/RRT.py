# Pieter-Jan Van de Maele 
import math
import numpy
from scipy.interpolate import interp1d

FAILURE = -1
REACHED = 1
ADVANCED = 2
TRAPPED = 3
q_new_global = None
footsize = 50

def euclidian_distance(node1, node2):
  return numpy.linalg.norm(node1.config - node2.config)

class Tree:
  def __init__(self, q_init):
    self.root = Node(q_init, self)
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
  
class Node:
  def __init__(self, q, tree=None):
    self.children = []
    self.parent = None
    self.config = q
    self.tree = tree

  def addChild(self, child):
    child.parent = self
    child.tree = self.tree
    self.children.append(child)
    if self.tree is not None:
      self.tree.lastNode = self
  
def CollisionFree(node, terrain):
  if node.config.item(1) < terrain.item(min(node.config.item(0), 499)) and \
     node.config.item(1) < terrain.item(min(node.config.item(0) + footsize/2.0, 499)) and \
     node.config.item(1) < terrain.item(min(node.config.item(0) - footsize/2.0, 499)):
    return True

# Normal RRT
# -------------------- 
def Build_RRT(q_init, numberOfNodes, delta_q, terrain):
  tree = Tree(q_init)
  for k in range(1, numberOfNodes):
    q_rand = numpy.array([numpy.random.randint(0, 500), numpy.random.randint(0, 500)])
    Extend(tree, q_rand, delta_q, terrain)

  return tree

def Extend(tree, q_rand, delta_q, terrain):
  randNode = Node(q_rand)
  near = tree.nearestNode(tree.root, randNode)
  
  u = randNode.config - near.config
  u = u/numpy.linalg.norm(u)
  
  q_new = near.config + u*delta_q
  global q_new_global   
  q_new_global = q_new

  newNode = Node(q_new) 
  if CollisionFree(newNode, terrain):
    near.addChild(newNode)
    if numpy.linalg.norm(q_new - q_rand) < delta_q:
      return REACHED
    else:
      return ADVANCED
  else:
    return TRAPPED

# RRT-Connect
# --------------------
def RRT_Connect_Planner(q_init, q_goal, numberOfNodes, delta_q, terrain):
  tree_a = Tree(q_init)
  tree_b = Tree(q_goal)

  tree_1 = tree_a
  tree_2 = tree_b

  i = 1
  for k in range(1, numberOfNodes):
    q_rand = numpy.array([numpy.random.randint(0, 500), numpy.random.randint(0, 500)])
    
    if(Extend(tree_1, q_rand, delta_q, terrain) is not TRAPPED):
      if(Connect(tree_2, q_new_global, delta_q, terrain) is REACHED):
        return Path(tree_1, tree_2)
      if i == 1:
        tree_1 = tree_b
        tree_2 = tree_a
      else:
        tree_1 = tree_a
        tree_2 = tree_b
    i *= -1
  
  return tree_a, tree_b, FAILURE
    
def Connect(tree, q, delta_q, terrain):
  s = ADVANCED
  while s is ADVANCED:
    s = Extend(tree, q, delta_q, terrain)

  return s
   
def Path(tree_a, tree_b):
  path_a = []
  node = tree_a.lastNode 
  while node is not None:
    path_a.append(node.config)
    node = node.parent
  
  path_b = []
  node = tree_b.lastNode 
  while node is not None:
    path_b.append(node.config)
    node = node.parent

  path_a.reverse()

  path_raw = path_a + path_b

  return tree_a, tree_b, path_raw


  
  

    
    
    

