#-----------------------------------------------------------------
#  Name: MATLIB Material Library Extension for VERTIS
#  Version: 1.0
#  Author: Caspar J. Anderegg - February 2012
#  
#  Purpose: Allow material and texture sampling on models via VERTIS.
#-----------------------------------------------------------------

from vectOps import *
import math, Image, os

# A simple class to hold a texture
class Texture:
  def __init__(self, name):
    # Load in the image
    self.path= name
    im= Image.open(name)
    # Get the image size
    self.size= im.size
    #Create the 
    self.t= im.load()

  # Given a pair of texture coordinates, return the associated color
  def color(self,u,v):
    # Compute x and y coordinates in the image
    x= u*self.size[0]
    y= v*self.size[1]
    x_lo= int(x)
    x_hi= x_lo + 1
    y_lo= int(y)
    y_hi= y_lo + 1
    # Compute the bilinear interpolation weights
    wx= x - float(x_lo)
    wy= y - float(y_lo)
    w1= (1.0-wx) * (1.0-wy)
    w2= (1.0-wx) * wy
    w3= wx * (1.0-wy)
    w4= wx * wy
    # Compute colors
    c1= self.t[x_lo%self.size[0], y_lo%self.size[1]] 
    c2= self.t[x_lo%self.size[0], y_hi%self.size[1]]
    c3= self.t[x_hi%self.size[0], y_lo%self.size[1]]
    c4= self.t[x_hi%self.size[0], y_hi%self.size[1]]
    # Compute r, g and b components
    r= int(w1*c1[0] + w2*c2[0] + w3*c3[0] + w4*c4[0]) 
    g= int(w1*c1[1] + w2*c2[1] + w3*c3[1] + w4*c4[1])
    b= int(w1*c1[2] + w2*c2[2] + w3*c3[2] + w4*c4[2])
    return (r,g,b)

# A simple class to hold a Material
class Material:
  def __init__(self,name):
    self.name= name
    self.diffuse= (0.0,0.0,0.0)
    self.specular= (0.0,0.0,0.0)
    self.ambient= (0.0,0.0,0.0)
    self.n= 0.0
    self.texture= None
    
  # Return the color of a point, given a pair of texture coordinates, a normal, and a light vector
  def shade(self,u,v,n,l, untextured):
    diff= max(dot(n,l),0)
    if self.texture==None or untextured:
      c= (self.diffuse[0]*256.0, self.diffuse[1]*256.0, self.diffuse[2]*256.0)
    else:
      c= self.texture.color(u,v)
    r= self.ambient[0]*256.0
    r+= diff*c[0]
    g= self.ambient[1]*256.0
    g+= diff*c[1]
    b= self.ambient[2]*256.0
    b+= diff*c[2]
    return min(int(r),255)<<16 | min(int(g),255)<<8 | min(int(b),255)
    
  # Return the unshaded color at a point
  def color(self,u,v,untextured):
    if self.texture!= None and not untextured:
      c= self.texture.color(u,v)
      r= self.ambient[0]*256.0 + c[0]
      g= self.ambient[1]*256.0 + c[1]
      b= self.ambient[2]*256.0 + c[2]
    else:
      r= (self.ambient[0] + self.diffuse[0])*256.0
      g= (self.ambient[1] + self.diffuse[1])*256.0
      b= (self.ambient[2] + self.diffuse[2])*256.0
    return min(int(r),255)<<16 | min(int(g),255)<<8 | min(int(b),255)
    
  #Convert to a string
  def __str__(self):
    return '<Material: %s; Ka: (%.4f,%.4f,%.4f); Kd: (%.4f,%.4f,%.4f); Ks: (%.4f,%.4f,%.4f); Ns: %.4f>' % (self.name, self.ambient[0], self.ambient[1], self.ambient[2], self.diffuse[0], self.diffuse[1], self.diffuse[2], self.specular[0], self.specular[1], self.specular[2], self.n)

# A simple class to hold a Material library
class MatLib:
  # Read in a material library given a file name
  def __init__(self, fname):
    self.materials= {}
    # Read file
    try:
      f= open(fname,'r')
      data= f.readlines()
      f.close()
    except IOError:
      print "ERROR: Unable to locate material library file"
      self.materials==None
      return
      
    # Parse Materials from library
    working= None
    for line in data:
       tokens= line.replace('\n','').replace('\r','').split(' ')
       while '' in tokens: tokens.remove('')
       # If a new material definition, 
       if len(tokens)==2 and tokens[0]=='newmtl':
         working= tokens[1]
         m= Material(working)
         self.materials[working]= m
       # Parse a diffuse color
       elif len(tokens)==4 and tokens[0]=='Kd' and not working==None:
         r= float(tokens[1])
         g= float(tokens[2])
         b= float(tokens[3])
         self.materials[working].diffuse= (r,g,b)
       # Parse a ambient color
       elif len(tokens)==4 and tokens[0]=='Ka' and not working==None:
         r= float(tokens[1])
         g= float(tokens[2])
         b= float(tokens[3])
         self.materials[working].ambient= (r,g,b)
       # Parse a specular color
       elif len(tokens)==4 and tokens[0]=='Ks' and not working==None:
         r= float(tokens[1])
         g= float(tokens[2])
         b= float(tokens[3])
         self.materials[working].specular= (r,g,b)
       # Parse a specular exponent
       elif len(tokens)==2 and tokens[0]=='Ns' and not working==None:
         self.materials[working].n= float(tokens[1])
       # Parse a texture
       elif len(tokens)==2 and tokens[0]=='map_Kd' and not working==None:
         try:
           path= os.path.join(os.path.dirname(fname), tokens[1])
           t= Texture(path)
           self.materials[working].texture= t
         except IOError:
           print "Texture loading failed:%s"%path
           self.materials[working].texture= None

