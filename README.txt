README.txt


1. OVERVIEW

ROS Package: cfg_obj
Version: 1.0 Alpha
Authors: Caspar Anderegg (cja58) and Bill Best (wpb47)
Created: 25 Apr 2012
Repository: git://github.com/canderegg/cfg_obj.git
Purpose: Includes code associated with object identification using context-free grammars.


2. INSTALLATION

The code can be downloaded with git using the command:

  git clone git://github.com/canderegg/cfg_obj.git

It requires the Point Cloud Library (PCL) ROS package, as well as the ROSPY package.
Once downloaded, it can be made using the command:

  rosmake cfg_obj


3. MODULES

The modular programs within the ROS package are listed below:

  scan - A virtual scanner to convert a 3D mesh in Wavefront Object (.obj/.mtl) format to a Point Cloud Data (.pcd) file.

  ransac - A generalized RANSAC algorithm to segment a Point Cloud Data (.pcd) file into component shapes.

For more information regarding the usage and options available in a module, type:
  
  rosrun cfg_obj [moduleName] --help


4. KNOWN BUGS

 - The virtual scanner calls a Python script. Filename arguments are passed to this script, and so relative path information is lost. When calling the virtual scanner, absolute paths should be used for all filename arguments.

 - The .pcd file output by the virtual scanner uses unsigned encodings for colors, rather than float encodings. This has been known to cause problems with some of the PCL methods for loading .pcd files. If this an issue the header can be changed from TYPE F F F U to TYPE F F F F. This will fix the problem but corrupt associated color information.

 - The ransac segmentation algorithm runs in O(m * 3^n) time, where m is the number of points and n is the smallest number of shapes that can accurately describe the model. For large or complex point clouds, this rapidly becomes prohibitive. Future work may include a different search method.


5. CONTACT INFO

Contact us at:
Caspar Anderegg (cja58@cornell.edu)
Bill Best (wpb47@cornell.edu)


6. AKNOWLEDGMENTS

Special thanks to:

  Ashutosh Saxena - Assistant Professor and course instructor CS4758, Cornell University
  
  Abhishek Anand - Ph.D. Mentor
