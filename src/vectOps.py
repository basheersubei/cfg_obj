#-----------------------------------------------------------------
#  Name: Vector Operations Library for VERTIS
#  Version: 1.0
#  Author: Caspar J. Anderegg - February 2012
#  
#  Purpose: Implement basic vector operations for VERTIS.
#-----------------------------------------------------------------
#!/usr/bin/env python

import math

# Vector addition
def add(v1,v2):
  return (v1[0]+v2[0],v1[1]+v2[1],v1[2]+v2[2])
  
# Return the L2 norm of the specified vector
def norm(v):
  return math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)
  
# Vector subtraction
def sub(v1,v2):
  return (v1[0]-v2[0],v1[1]-v2[1],v1[2]-v2[2])
  
# Vector dot products
def dot(v1,v2):
  return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2]
  
# Vector cross products
def cross(v1,v2):
  x= v1[1]*v2[2] - v1[2]*v2[1]
  y= v1[2]*v2[0] - v1[0]*v2[2]
  z= v1[0]*v2[1] - v1[1]*v2[0]
  return (x,y,z)
