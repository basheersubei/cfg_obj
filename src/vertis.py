#-----------------------------------------------------------------
#  Name: VERTIS Virtual Scanner
#  Version: 2.0
#  Author: Caspar J. Anderegg - February 2012
#  
#  Purpose: To convert 3D models in OBJ format into pointcloud 
#      files in PCD format.
#-----------------------------------------------------------------

#!/usr/bin/env python
import os, sys, math, time, random, matlib
from vectOps import *

header = "# .PCD v.7 - Point Cloud Data file format\n\
FIELDS x y z rgb\n\
SIZE 4 4 4 4\n\
TYPE F F F U\n\
COUNT 1 1 1 1\n\
WIDTH XXXX\n\
HEIGHT 1\n\
VIEWPOINT 0 0 0 1 0 0 0\n\
POINTS XXXX\n\
DATA ascii\n"
version= "2.0"
EPSILON= 0.000001
KINECT_DEGRADE= 0.0035
GRAY= 75
DEFAULT_COLOR= GRAY*2**16 + GRAY*2**8 + GRAY

VERTICES= {}
V_NORMS= {}
V_TEX= {}
FACES= {}

CENTROID= [0,0,0]

# A Ray structure composed of an origina and a direction
class Ray:
  def __init__(self, origin, direction):
    self.origin= (origin[0]+EPSILON*direction[0],origin[1]+EPSILON*direction[1],
        origin[2]+EPSILON*direction[2])
    self.direction= direction

# A class to hold the information about each face
class Face:
  # Construct a face given a line of lexical tokens
  def __init__(self,vertices,v_tex,v_norms,material):
    # Split vertices from normals and texture coordinates
    self.vertices= vertices
    self.v_tex= v_tex
    self.v_norms= v_norms
    self.material= material
    
    # Compute a normal for the plane
    global VERTICES, WINDING
    v1= VERTICES[self.vertices[0]]
    v2= VERTICES[self.vertices[1]]
    v3= VERTICES[self.vertices[2]]
    ab= sub(v1,v3)
    bc= sub(v2,v3)
    if not WINDING:
      self.normal= cross(ab,bc)
    else:
      self.normal= cross(bc,ab)
    n= norm(self.normal)
    if not n==0: self.normal= (self.normal[0]/n,self.normal[1]/n,self.normal[2]/n)
    else: self.normal= (1,0,0)
     
    # Compute the bounding box of this face
    self.low_bound= (min([v1[0],v2[0],v3[0]])-EPSILON/10.0,min([v1[1],v2[1],v3[1]])
        -EPSILON/10.0,min([v1[2],v2[2],v3[2]])-EPSILON/10.0)
    self.hi_bound= (max([v1[0],v2[0],v3[0]])+EPSILON/10.0,max([v1[1],v2[1],v3[1]])
        +EPSILON/10.0, max([v1[2],v2[2],v3[2]])+EPSILON/10.0)
        
    # Compute the centroid of this face
    self.centroid= ((v1[0]+v2[0]+v3[0])/3.0, (v1[0]+v2[0]+v3[0])/3.0,
        (v1[0]+v2[0]+v3[0])/3.0)
        
  # A method to check intersection between a ray and this face
  def intersect(self, ray):
    global VERTICES
    # compute point of intersection with the plane
    RdotN= dot(ray.direction,self.normal)
    P0= VERTICES[self.vertices[0]]
    Diff= sub(P0,ray.origin)
    DiffdotN= dot(self.normal,Diff)
    t= DiffdotN / RdotN
    # If intersection is behind ray, return false
    if t<0: return False
    P= add(ray.origin,(t*ray.direction[0],t*ray.direction[1],t*ray.direction[2]))
    # check if point is in the plane
    v1= VERTICES[self.vertices[0]]
    v2= VERTICES[self.vertices[1]]
    v3= VERTICES[self.vertices[2]]
    if dot(cross(sub(v2,v1),sub(P,v1)),self.normal) <0: return False
    elif dot(cross(sub(v3,v2),sub(P,v2)),self.normal) <0: return False
    elif dot(cross(sub(v1,v3),sub(P,v3)),self.normal) <0: return False
    else: return True
    
  # A method to check intersection between a ray and this faces bounding volume
  def intersectBV(self,ray):
    # Intersect x slab
    if ray.direction[0]==0:
      if ray.origin[0]<self.low_bound[0] or ray.origin[0]>self.hi_bound[0]:
        return False
      else:
        x1= 0
        x2= 2*max([SIZE[0][0]-SIZE[0][1],SIZE[1][0]-SIZE[1][1],SIZE[2][0]-SIZE[2][1]])
    else:
      # find intersections
      x1= (self.low_bound[0]-ray.origin[0])/ray.direction[0]
      x2= (self.hi_bound[0]-ray.origin[0])/ray.direction[0]
      # if reversed, swap them
      if x1 > x2: tmp= x1; x1= x2; x2= tmp
    # Intersect y slab
    if ray.direction[1]==0:
      if ray.origin[1]<self.low_bound[1] or ray.origin[1]>self.hi_bound[1]:
        return False
      else:
        y1= 0
        y2= 2*max([SIZE[0][0]-SIZE[0][1],SIZE[1][0]-SIZE[1][1],SIZE[2][0]-SIZE[2][1]])
    else:
      # find intersections
      y1= (self.low_bound[1]-ray.origin[1])/ray.direction[1]
      y2= (self.hi_bound[1]-ray.origin[1])/ray.direction[1]
      # if reversed, swap them
      if y1 > y2: tmp= y1; y1= y2; y2= tmp
    # If no overlap, return false
    if max([x1,y1]) >= min([x2, y2]): return False
    # Intersect z slab
    if ray.direction[2]==0:
      if ray.origin[2]<self.low_bound[2] or ray.origin[2]>self.hi_bound[2]:
        return False
      else:
        z1= 0
        z2= 2*max([SIZE[0][0]-SIZE[0][1],SIZE[1][0]-SIZE[1][1],SIZE[2][0]-SIZE[2][1]])
    else:
      # find intersections
      z1= (self.low_bound[2]-ray.origin[2])/ray.direction[2]
      z2= (self.hi_bound[2]-ray.origin[2])/ray.direction[2]
      # if reversed, swap them
      if z1 > z2: tmp= z1; z1= z2; z2= tmp
    # If intersection behind the ray, return false
    if min([x2, y2, z2]) < 0: return False
    # If no overlap, return false
    if max([x1,y1,z1]) >= min([x2, y2, z2]): return False
    # Otherwise return true
    return True
    
# A recursive bounding volume
class BVol:
  # Create a new bounding volume with the specified children
  def __init__(self, LeftChild, RightChild):
    self.left= LeftChild
    self.right= RightChild
    # Compute the bounding box for this BVol
    a= self.left.low_bound
    b= self.right.low_bound
    self.low_bound= (min([a[0],b[0]]),min([a[1],b[1]]),min([a[2],b[2]]))
    a= self.left.hi_bound
    b= self.right.hi_bound
    self.hi_bound= (max([a[0],b[0]]),max([a[1],b[1]]),max([a[2],b[2]]))
    # Compute the centroid for this BVol
    c1= self.left.centroid
    c2= self.right.centroid
    total= add(c1,c2)
    self.centroid= (total[0]/2.0, total[1]/2.0, total[2]/2.0)
  
  # An intersection method for this bounding volume
  def intersectBV(self,ray):
    # Intersect x slab
    if ray.direction[0]==0:
      if ray.origin[0]<self.low_bound[0] or ray.origin[0]>self.hi_bound[0]:
        return False
      else:
        x1= 0
        x2= 2*max([SIZE[0][0]-SIZE[0][1],SIZE[1][0]-SIZE[1][1],SIZE[2][0]-SIZE[2][1]])
    else:
      # find intersections
      x1= (self.low_bound[0]-ray.origin[0])/ray.direction[0]
      x2= (self.hi_bound[0]-ray.origin[0])/ray.direction[0]
      # if reversed, swap them
      if x1 > x2: tmp= x1; x1= x2; x2= tmp
    # Intersect y slab
    if ray.direction[1]==0:
      if ray.origin[1]<self.low_bound[1] or ray.origin[1]>self.hi_bound[1]:
        return False
      else:
        y1= 0
        y2= 2*max([SIZE[0][0]-SIZE[0][1],SIZE[1][0]-SIZE[1][1],SIZE[2][0]-SIZE[2][1]])
    else:
      # find intersections
      y1= (self.low_bound[1]-ray.origin[1])/ray.direction[1]
      y2= (self.hi_bound[1]-ray.origin[1])/ray.direction[1]
      # if reversed, swap them
      if y1 > y2: tmp= y1; y1= y2; y2= tmp
    # If no overlap, return false
    if max([x1,y1]) >= min([x2, y2]): return False
    # Intersect z slab
    if ray.direction[2]==0:
      if ray.origin[2]<self.low_bound[2] or ray.origin[2]>self.hi_bound[2]:
        return False
      else:
        z1= 0
        z2= 2*max([SIZE[0][0]-SIZE[0][1],SIZE[1][0]-SIZE[1][1],SIZE[2][0]-SIZE[2][1]])
    else:
      # find intersections
      z1= (self.low_bound[2]-ray.origin[2])/ray.direction[2]
      z2= (self.hi_bound[2]-ray.origin[2])/ray.direction[2]
      # if reversed, swap them
      if z1 > z2: tmp= z1; z1= z2; z2= tmp
    # If intersection behind the ray, return false
    if min([x2, y2, z2]) < 0: return False
    # If no overlap, return false
    if max([x1,y1,z1]) >= min([x2, y2, z2]): return False
    # Otherwise return true
    return True
    
  # A method to check if the given ray is occluded by any face in the hierarchy
  def occluded(self, ray):
    if not self.intersectBV(ray): return False
    else:
      if self.left.__class__.__name__ == 'Face':
        hitLeft= self.left.intersectBV(ray)
        if self.right.__class__.__name__=='Face':
          hitRight= self.right.intersectBV(ray)
          if not hitLeft and not hitRight:
            return False
          elif not hitLeft:
            return self.right.intersect(ray)
          elif not hitRight:
            return self.left.intersect(ray)
          else:
            return self.right.intersect(ray) or self.left.intersect(ray)
        else:
          if not hitLeft:
            return self.right.occluded(ray)
          else:
            return self.left.intersect(ray) or self.right.occluded(ray)
      elif self.right.__class__.__name__=='Face':
        hitRight= self.right.intersectBV(ray)
        if not hitRight:
          return self.left.occluded(ray)
        else:
          return self.right.intersect(ray) or self.left.occluded(ray)
      else:
        return self.left.occluded(ray) or self.right.occluded(ray)
        
# Construct a logical bounding volume hierarchy given a list of faces
def constructBVH():
  global FACES, BVH, VERBOSE
  # Establish working sets
  working= []
  for f in FACES: working.append(f)
  i=0
  while len(working)>1:
    if VERBOSE: print "    Clustering tier %i..."%i
    # Tier   
    tier= {}
    updated= []
    # Compute the centroid distances between every pair of objects
    pairs= []
    for ind1 in range(1,len(working)):
      o1= working[ind1]
      for ind2 in range(0,ind1):
        o2= working[ind2]
        diff= sub(o1.centroid,o2.centroid)
        distance= dot(diff,diff)
        pairs.append((distance,(o1,o2)))
    # Sort list given above
    if VERBOSE: print "      %i candidate clusters computed..." % len(pairs)
    pairs= sorted(pairs, key=lambda tup: tup[0])
    if VERBOSE: print "      Candidates sorted..."
    # Cluster into a BVs using the information above
    for j in range(0,len(pairs)):
      (d,(p1,p2))= pairs[j]
      if p1 in tier or p2 in tier:
        pass
      else:
        BV= BVol(p1,p2)
        updated.append(BV)
        tier[p1]= 0
        tier[p2]= 0
        if len(tier)==len(working): break
        
    for p in working:
      if not p in tier: updated.append(p)
    # Swap out working copy
    working= updated
    i+=1
    
  # Set the final bounding volume hierarchy
  for bv in working: BVH= bv     
    
# Check is point is occluded by another face
def occluded(point):
  global BVH
  r= Ray(point, (-VIEW[0], -VIEW[1], -VIEW[2]))
  if BVH.__class__.__name__=='Face':
    return BVH.intersect(r)
  else: return BVH.occluded(r)
  
# Find the point in the model nearest to the camera plane
def findBasis():
  global VERTICES, VIEW, basis
  if VIEW==None: basis= None; return
  antiView= (-VIEW[0], -VIEW[1], -VIEW[2])
  basis= VERTICES[1]
  for i in range(2,len(VERTICES)):
    v= VERTICES[i]
    difference= sub(basis,v)
    cosT= dot(difference, antiView)
    if cosT<0: basis= v    

# The primary method invoked to convert a obj file to a pcd file
def convertOBJtoPCD(mesh_file, pcd_file):
  global IDEAL_SAMPLE, SAMPLE_RATE, ESTIMATE, VERBOSE, V_NORMS, V_TEX
  global VERTICES, FACES, header, version, GAUSSIAN, CENTROID, SIZE, VIEW
  global BACKFACE, OCCLUDE, basis, KINECT, SCALE, MatLib, SHADE, RESIZE
  global UNTEX
  
  if VIEW!=None: LIGHT= sub((0,0,0), VIEW)
  else: SHADE= False

  print "\nScan initiated..."
  start= time.time()

  # Instantiate requisite pointers
  input= open(mesh_file,'r')
  tmpName= '%f.tmp' % time.time()
  tmp= open(tmpName, 'w')
  vert_counter= 0
  tex_counter= 0
  norms_counter= 0
  points_counter= 0
  

  #Structure to keep track of model size
  SIZE= [[0,0],[0,0],[0,0]]

  if VERBOSE: print "  Reading OBJ file..."
  # Read OBJ file
  workingMaterial= None
  for s in input.xreadlines():
    # Split line into tokens
    tokens= s.replace('\n','').replace('\r','').split(' ')
    while '' in tokens: tokens.remove('')
    # If line describes a vertex, take it
    if len(tokens)>3 and tokens[0]=='v':
      # Add vertex to vertex list
      x= float(tokens[1])
      y= float(tokens[2])
      z= float(tokens[3])
      vert_counter+=1
      VERTICES[vert_counter]=(x,y,z)
      # Update model size
      if x>SIZE[0][0]: SIZE[0][0]= x
      elif x<SIZE[0][1]: SIZE[0][1]= x
      if y>SIZE[1][0]: SIZE[1][0]= y
      elif y<SIZE[1][1]: SIZE[1][1]= y
      if z>SIZE[2][0]: SIZE[2][0]= z
      elif z<SIZE[2][1]: SIZE[2][1]= z
      # Update model centroid
      CENTROID[0]= (CENTROID[0]*float(vert_counter-1)+x)/float(vert_counter) 
      CENTROID[1]= (CENTROID[1]*float(vert_counter-1)+y)/float(vert_counter) 
      CENTROID[2]= (CENTROID[2]*float(vert_counter-1)+z)/float(vert_counter) 
      
    # If line describes a texture coordinate
    elif len(tokens)>2 and tokens[0]=='vt':
      # Add coordinates to texture list
      x= float(tokens[1])
      y= float(tokens[2])
      try:
        z= float(tokens[3])
      except IndexError: z= None
      tex_counter+=1
      V_TEX[tex_counter]=(x,y,z)
      
    # If line describes a vertex normal
    elif len(tokens)>3 and tokens[0]=='vn':
      # Add normal to normal list
      x= float(tokens[1])
      y= float(tokens[2])
      z= float(tokens[3])
      norms_counter+=1
      V_NORMS[norms_counter]=(x,y,z)
      
    # If line describes a face, parse and construct
    elif len(tokens)>3 and tokens[0]=='f':
      # Split vertices from normals
      for (i,t) in enumerate(tokens):
        tokens[i]= t.strip().split('/')
      while [] in tokens: tokens.remove([])
      # Append triangular faces to list of faces
      for i in range(2,len(tokens)-1):
        # Convert vertices
        vertices= (int(tokens[1][0]),int(tokens[i][0]),int(tokens[i+1][0]))
        #Convert texture coordinates
        try:
          v_tex= (int(tokens[1][1]),int(tokens[i][1]),int(tokens[i+1][1]))
        except IndexError:
          v_tex= None
        except ValueError:
          v_tex= None
        # Convert vertex normals
        try:
          v_norms= (int(tokens[1][2]),int(tokens[i][2]),int(tokens[i+1][2]))
        except IndexError:
          v_norms= None
        except ValueError:
          v_norms= None
        # Construct triangular face
        if MatLib==None or workingMaterial==None: m= None
        else: m= workingMaterial
        face= Face(vertices,v_tex,v_norms,m)
        FACES[face]=0
  
    # If line describes a material library, load
    elif len(tokens)==2 and tokens[0]=='mtllib':
      fname= tokens[1]
      path= os.path.join(os.path.dirname(mesh_file),fname)
      MatLib= matlib.MatLib(path)
  
    # If line describes a material, set working
    elif len(tokens)==2 and tokens[0]=='usemtl':
      workingMaterial= tokens[1]
        
  input.close()
  if VERBOSE: print "    Input file read."

  # If necessary, estimate a good sample rate
  dist= max([SIZE[0][0]-SIZE[0][1], SIZE[1][0]-SIZE[1][1], SIZE[2][0]-SIZE[2][1]])
  if ESTIMATE:
    SAMPLE_RATE= float(IDEAL_SAMPLE)/dist  
  
  # If necessary, compute the model scale rate
  if not RESIZE==None:
    scale_rate= RESIZE / dist
  else:
    scale_rate= 1.0
  
  # If necessary, find a kinect basis framework
  if not KINECT==None:
    if VERBOSE: print "  Calibrating kinect framework..."
    findBasis()
    if VERBOSE: print "    Kinect calibrated."      
  
  # If necessary, compute gaussian perturbation rate
  if not GAUSSIAN==0:
    GAUSSIAN= (GAUSSIAN/100.0)*dist/math.sqrt(3)
    
  # Cull backfaces from the list of faces if required
  if BACKFACE and VIEW!=None:
    if VERBOSE: print "  Culling backfaces..."
    remove= {}
    for f in FACES:
      if f.normal[0]*VIEW[0] + f.normal[1]*VIEW[1] + f.normal[2]*VIEW[2] >= -EPSILON:
        remove[f]= 0
    for f in remove: del FACES[f]
    if VERBOSE: print "    Backfaces removed."
    
  # Cull occluded faces from the list of faces
  if OCCLUDE and VIEW!=None:
    if VERBOSE: print "  Culling occluded faces..."
    remove= {}
    l= float(len(FACES))
    for (i,f) in enumerate(FACES):
      v1= VERTICES[f.vertices[0]]
      r1= Ray(v1, (-VIEW[0], -VIEW[1], -VIEW[2]))
      v2= VERTICES[f.vertices[1]]
      r2= Ray(v2, (-VIEW[0], -VIEW[1], -VIEW[2]))
      v3= VERTICES[f.vertices[2]]
      r3= Ray(v3, (-VIEW[0], -VIEW[1], -VIEW[2]))
      v1_hidden= False
      v2_hidden= False
      v3_hidden= False
      for fprime in FACES:
        if fprime.intersectBV(r1):
          v1_hidden= v1_hidden or fprime.intersect(r1)
        if fprime.intersectBV(r2):
          v2_hidden= v2_hidden or fprime.intersect(r2)
        if fprime.intersectBV(r3):
          v3_hidden= v3_hidden or fprime.intersect(r3)
        if v1_hidden and v2_hidden and v3_hidden: break
      if v1_hidden and v2_hidden and v3_hidden: remove[f]= 0
      if VERBOSE: print "    %.2f percent complete..." % (float(i)/l * 100.0)
    for f in remove: del FACES[f]
    if VERBOSE: print "    Occluded faces removed."
    
  # if necessary, construct a BVH
  if TRACE:
    if VERBOSE: print "  Building a bounding volume hierarchy..."
    constructBVH()
    if VERBOSE: print "    Bounding volume hierarchy built."

  # Interpolate points across each face
  if VERBOSE: print "  Sampling faces..."
  numFaces= float(len(FACES))

  for (f,face) in enumerate(FACES):
    # Locate relevant vertices
    v1= VERTICES[face.vertices[0]]
    v2= VERTICES[face.vertices[1]]
    v3= VERTICES[face.vertices[2]]
    # Locate relevant texture coordinates
    if face.v_tex!=None:
      t1= V_TEX[face.v_tex[0]]
      t2= V_TEX[face.v_tex[1]]
      t3= V_TEX[face.v_tex[2]]
    # Compute face sidelengths
    d13= math.sqrt((v1[0]-v3[0])**2 + (v1[1]-v3[1])**2 + (v1[2]-v3[2])**2)
    d23= math.sqrt((v2[0]-v3[0])**2 + (v2[1]-v3[1])**2 + (v2[2]-v3[2])**2)
    #Compute samples and sample step per vector
    samples1= d13*float(SAMPLE_RATE)
    step1= 1.0 / samples1
    samples2= d23*float(SAMPLE_RATE)
    step2= 1.0 / samples2
    # Write face vertices into temp file
    if not TRACE or not occluded(v1):
      bytes= DEFAULT_COLOR
      if MatLib!=None and face.material!=None:
        if face.v_tex!=None:
          u= t1[0]
          v= t1[1]
        else: u=0; v=0;
        try:
          if SHADE: bytes= MatLib.materials[face.material].shade(u,v,face.normal,LIGHT,UNTEX)
          else: bytes= MatLib.materials[face.material].color(u,v,UNTEX)
        except KeyError: pass
      tmp.write("%f %f %f %i\n"%(v1[0]*scale_rate,v1[1]*scale_rate,v1[2]*scale_rate,bytes))
      points_counter+=1
    if not TRACE or not occluded(v2):
      bytes= DEFAULT_COLOR
      if MatLib!=None and face.material!=None:
        if face.v_tex!=None:
          u= t2[0]
          v= t2[1]
        else: u=0; v=0;
        try:
          if SHADE: bytes= MatLib.materials[face.material].shade(u,v,face.normal,LIGHT,UNTEX)
          else: bytes= MatLib.materials[face.material].color(u,v,UNTEX)
        except KeyError: pass
      tmp.write("%f %f %f %i\n"%(v2[0]*scale_rate,v2[1]*scale_rate,v2[2]*scale_rate,bytes))
      points_counter+=1 
    # Interpolate over face using barycentric coordinates
    for i in range(0,int(samples1)):
      coord1= step1*float(i)
      for j in range(0,int((1.0-coord1)/step2)):
        coord2= step2*float(j)
        coord3= 1.0 - coord1 - coord2
        x= coord1*v1[0] + coord2*v2[0] + coord3*v3[0] + random.gauss(0,GAUSSIAN)
        y= coord1*v1[1] + coord2*v2[1] + coord3*v3[1] + random.gauss(0,GAUSSIAN)
        z= coord1*v1[2] + coord2*v2[2] + coord3*v3[2] + random.gauss(0,GAUSSIAN)
        
        # If required, compute a kinect perturbation rate
        if not KINECT==None:
          distance= sub((x,y,z),basis)
          proj= dot(VIEW,distance)/dist*SCALE
          total= KINECT+proj
          perturbation= total**2 * KINECT_DEGRADE
          d= random.gauss(0,perturbation)
          x+= d*VIEW[0]
          y+= d*VIEW[1]
          z+= d*VIEW[2]

        # If available, compute color
        bytes= DEFAULT_COLOR
        if MatLib!=None and face.material!=None:
          if face.v_tex!=None:
            u= coord1*t1[0] + coord2*t2[0] + coord3*t3[0]
            v= coord1*t1[1] + coord2*t2[1] + coord3*t3[1]
          else: u=0; v=0;
          try:
            if SHADE: bytes= MatLib.materials[face.material].shade(u,v,face.normal,LIGHT,UNTEX)
            else: bytes= MatLib.materials[face.material].color(u,v, UNTEX)
          except KeyError: pass
        if not TRACE or not occluded((x,y,z)):
          tmp.write("%f %f %f %i\n"%(x*scale_rate,y*scale_rate,z*scale_rate,bytes))
          points_counter+=1
    if VERBOSE: print "      Sampling %.2f percent complete..." % (float(f+1)/numFaces * 100.0)
  tmp.close()
  if VERBOSE: print "    Sampling complete."

  # Write PCD file
  if VERBOSE: print "  Writing back..."
  output= open(pcd_file,'w')
  tmp= open(tmpName, 'r')
  header= header.replace('XXXX', str(points_counter))
  output.write(header)
  for s in tmp.xreadlines():
    output.write(s)
  tmp.close()
  output.close()
  if VERBOSE: print "    Writing complete."

  # Clean up temp
  if VERBOSE: print "  Clearning up..."
  os.system('rm %s' % tmpName)
  if VERBOSE: print "    Temp files removed."
  
  print "Done. Scanned %i points in %i seconds.\n" % (points_counter,time.time()-start)

# Print the usage for this command line tool
def usage():
  print "\nUsage: rosrun vertis scan [options] <input_mesh.obj> <output_cloud.pcd>"
  print "\n  General options:"
  print "    --help (-h)         Print usage options."
  print "    -resize (-rs) N     Resize the models larges dimension to N meters"
  print "    -rewind (-rw)       Swap the winding direction (for computing normals)."
  print "    -verbose (-vv)      Verbose mode. Print completion status at each step."
  print "\n  Sampling rate:"
  print "    -autoset (-A) N     Optimize the sample rate to get N points along the largest dimension."
  print "    -sample (-s) N      Manually set the sample rate to some integer N."
  print "\n  Noise overlays:"
  print "    -calibrate (-c) N   Calibrate space so the model's largest dimension is length N meters."
  print "    -gaussian (-g) N    Add N percent Gaussian distortion to each sampled point."
  print "                          N must be a decimal number in the range [0,100]."
  print "    -kinect (-k) N      Simulate noise from a kinect camera distance N meters away from the model."
  print "                          (Requires -c and -V options as well)."
  print "\n  Viewpoint culling:"
  print "    -backface (-b)       Cull faces that point away from the view vector."
  print "                           (Requires -V option as well)."
  print "    -occlude (-o)        Cull faces where no vertex is visible to the camera."
  print "                           (Requires -V option as well)."
  print "    -trace (-tr)         Remove points not visible to the camera."
  print "                           (Requires -V option as well)."
  print "    -view (-V) \"<i,j,k>\" Set the view direction to be vector <i,j,k>."
  print "\n  Color Information:"
  print "    -shade (-sh)         Use Lambertian shading to color the material points."
  print "                           (Requires -V option as well)."
  print "    -untexture (-ut)     Disregard texure information when scanning."
  print  

if __name__ == "__main__":
  global VERBOSE, SAMPLE_RATE, ESTIMATE, IDEAL_SAMPLE, GAUSSIAN, VIEW
  global BACKFACE, OCCLUDE, WINDING, SCALE, KINECT, SHADE, TRACE, RESIZE
  global UNTEX
  
  print "\nVERTIS Virtual Scanner v %s" % version
  print "  written by Caspar Anderegg"

  # Usage definitions
  if (sys.argv.__len__() < 3 or '--help' in sys.argv 
      or '-h' in sys.argv):
    usage()
    sys.exit(2)
    
  # Verbose mode
  if '-verbose' in sys.argv or '-vv' in sys.argv: 
    VERBOSE= True
    print "\n  Setting Parameters..."
  else: VERBOSE= False

  # Resizing mode
  if '-resize' in sys.argv:
    RESIZE= float(sys.argv[sys.argv.index('-resize')+1])
    if VERBOSE: print "    Resize to: %.2f meters" % RESIZE
  elif '-rs' in sys.argv:
    RESIZE= float(sys.argv[sys.argv.index('-rs')+1])
    if VERBOSE: print "    Resize to: %.2f meters" % RESIZE
  else: RESIZE= None
     
  # Swap the winding direction mode
  if '-rewind' in sys.argv or '-rw' in sys.argv: 
    WINDING= True
    if VERBOSE: print "    Winding: Reversed"
  else: WINDING= False
  
  # Calibrate model size  
  if '-calibrate' in sys.argv: 
    SCALE= float(sys.argv[sys.argv.index('-calibrate')+1])
    if VERBOSE: print "    Calibration Scale: %.2f m" % SCALE
  elif '-c' in sys.argv: 
    SCALE= float(sys.argv[sys.argv.index('-c')+1])
    if VERBOSE: print "    Calibration Scale: %.2f m" % SCALE
  else: SCALE=1.0
   
  # Autoset sampling rate
  if '-autoset' in sys.argv: 
    ESTIMATE= True
    IDEAL_SAMPLE= float(sys.argv[sys.argv.index('-autoset')+1])
    if VERBOSE: print "    Sample Rate: Auto Scaled"
  elif '-A' in sys.argv: 
    ESTIMATE= True
    IDEAL_SAMPLE= float(sys.argv[sys.argv.index('-A')+1])
    if VERBOSE: print "    Sample Rate: Auto Scaled"
  else: 
    ESTIMATE= False
    IDEAL_SAMPLE= 500
    
  # Manual sampling rate  
  if '-sample' in sys.argv: 
    SAMPLE_RATE= float(sys.argv[sys.argv.index('-sample')+1])
    if VERBOSE: print "    Sample Rate: %.2f Manual" % SAMPLE_RATE
  elif '-s' in sys.argv: 
    SAMPLE_RATE= float(sys.argv[sys.argv.index('-s')+1])
    if VERBOSE: print "    Sample Rate: %.2f Manual" % SAMPLE_RATE
  else: SAMPLE_RATE= 5.0
  
  # Gaussian noise overlay
  if '-gaussian' in sys.argv: 
    GAUSSIAN= float(sys.argv[sys.argv.index('-gaussian')+1])
    if VERBOSE: print "    Noise: %.2f percent Gaussian" % GAUSSIAN
  elif '-g' in sys.argv: 
    GAUSSIAN= float(sys.argv[sys.argv.index('-g')+1])
    if VERBOSE: print "    Noise: %.2f percent Gaussian" % GAUSSIAN
  else: GAUSSIAN=0
  
  # Kinnect noise overlay
  if '-kinect' in sys.argv: 
    KINECT= float(sys.argv[sys.argv.index('-kinect')+1])
    if VERBOSE: print "    Noise: Kinect (distance %.2f m)" % KINECT
  elif '-k' in sys.argv: 
    KINECT= float(sys.argv[sys.argv.index('-k')+1])
    if VERBOSE: print "    Noise: Kinect (distance %.2f m)" % KINECT
  else: KINECT=None
  
  # Enable backface culling
  if '-backface' in sys.argv or '-b' in sys.argv:
    BACKFACE= True
    if VERBOSE: print "    Backface Culling: ON"
  else: BACKFACE= False
  
  # Enable occlusion culling
  if '-occlude' in sys.argv or '-o' in sys.argv:
    OCCLUDE= True
    if VERBOSE: print "    Occlusion Culling: ON"
  else: OCCLUDE= False
  
  # Enable trace culling
  if '-trace' in sys.argv or '-tr' in sys.argv:
    TRACE= True
    if VERBOSE: print "    Trace Point Removal: ON"
  else: TRACE= False

  # Set the viewpoint
  if '-view' in sys.argv:
    string= sys.argv[sys.argv.index('-view')+1]
    tokens= string[1:-1].strip().split(',')
    VIEW= (float(tokens[0]),float(tokens[1]),float(tokens[2]))
    n= norm(VIEW)
    VIEW= (VIEW[0]/n,VIEW[1]/n,VIEW[2]/n)
    if VERBOSE: print "    View Direction: <%.2f,%.2f,%.2f>" % VIEW
  elif '-V' in sys.argv: 
    string= sys.argv[sys.argv.index('-V')+1]
    tokens= string[1:-1].strip().split(',')
    VIEW= (float(tokens[0]),float(tokens[1]),float(tokens[2]))
    n= norm(VIEW)
    VIEW= (VIEW[0]/n,VIEW[1]/n,VIEW[2]/n)
    if VERBOSE: print "    View Direction: <%.2f,%.2f,%.2f>" % VIEW
  else: VIEW=(-1,-1,-1)
  
  # Enable occlusion culling
  if '-shade' in sys.argv or '-sh' in sys.argv:
    SHADE= True
    if VERBOSE: print "    Lambertian Shading: ON"
  else: SHADE= False
 
  # Disable texture mapping
  if '-untexture' in sys.argv or '-ut' in sys.argv:
    UNTEX= True
    if VERBOSE: print "    Texture Mapping: OFF"
  else: UNTEX= False

  # Run the virtual scanner
  mesh_file = sys.argv[-2]
  pcd_file = sys.argv[-1]
  convertOBJtoPCD(mesh_file, pcd_file)

