import numpy
import math

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

