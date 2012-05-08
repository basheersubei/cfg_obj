// Title: ransac.cpp
// Authors: Caspar Anderegg and Bill Best - Spring 2012
// Purpose: To implement a generalized multi-shape ransac algorithm for segmenting PCL point clouds

// Include the necessary standard libraries
#include <iostream>
#include <cstdlib>
#include <queue>
#include <cstring>
#include <cmath>
#include <string>
#include <fstream>
// Include necessary ros/pcl libraries
//#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
//#include <pcl_visualization/pcl_visualizer.h> //Modified for diamondback #include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <boost/thread/thread.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
// Include our own header files
#include "obj_point_types.h"

// Using declarations
using std::cout;
using std::vector;
using std::queue;
using std::flush;
using std::string;
using std::endl;
using std::ios;

// Set constants
int MAX_OUTLIERS;
const float PERCENT_OUTLIERS= .025;
const float THRESHOLD= 0.0025;
const float CYL_THRESH_MOD= 5;
int CLUSTERS= 0;
const int SHAPES= 3;
const int MAX_CYL_ITERATIONS= 250;
bool VERBOSE= false;
bool VISUAL= false;

// A tree structure to keep track of the shape sequences ----------------------------------------------------------
struct Tree {
  bool isRoot;
  int shape; //0 - plane; 1 - cylinder; 2 - sphere;
  vector<int>* outliers;
  int score;
  struct Tree* parent;
  vector<struct Tree*>* children;
};

typedef struct Tree treeNode;
//-----------------------------------------------------------------------------------------------------------------

/*
// Cloud Visualization Methods ---------------------------------------------------------------------------------------------
// Open 3D viewer to visualize the segmented pointcloud
boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  for (int i=0; i<(int)clouds.size(); i++){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cld (new pcl::PointCloud<pcl::PointXYZRGB>);
    cld->points.resize(clouds[i]->size());
    unsigned int color= (unsigned int)(((rand() % 256)<<16) | ((rand() % 256)<<8) | (rand() % 256));
    for (int j=0; j<(int)cld->points.size(); j++){
      cld->points[j].x= clouds[i]->points[j].x;
      cld->points[j].y= clouds[i]->points[j].y;
      cld->points[j].z= clouds[i]->points[j].z;
      cld->points[j].rgba= color;
    }
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cld);
    char label[50];
    sprintf(label, "cloud %d%s", i, "\0");
    viewer->addPointCloud<pcl::PointXYZRGB> (cld, rgb, label);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, label);
  }
  viewer->initCameraParameters ();
  return (viewer);
}

// Visualize the point clouds in different colors
void visualizeCloud(vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds){
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  viewer= rgbVis(clouds);
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}*/

// RANSAC Methods ---------------------------------------------------------------------------------------------
// Compute the best ransac fit for a single plane
vector<int> ransacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr updated){
  vector<int> inliers;
  // created RandomSampleConsensus object and compute the appropriated model
  pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
      model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (updated));
  pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
  ransac.setDistanceThreshold (THRESHOLD);
  ransac.computeModel();
  ransac.getInliers(inliers);

  return inliers;
}

// Compute the best ransac fit for a single sphere
vector<int> ransacSphere(pcl::PointCloud<pcl::PointXYZ>::Ptr updated){
  vector<int> inliers;
  // created RandomSampleConsensus object and compute the appropriated model
  pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr
      model_p (new pcl::SampleConsensusModelSphere<pcl::PointXYZ> (updated));
  pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
  ransac.setDistanceThreshold (THRESHOLD);
  ransac.computeModel();
  ransac.getInliers(inliers);

  return inliers;
}

// Compute the best ransac fit for a single cylinder
vector<int> ransacCylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr updated, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals){
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
  pcl::PointIndices::Ptr inliers_cyl (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);

  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.1);
  seg.setMaxIterations (MAX_CYL_ITERATIONS);
  seg.setDistanceThreshold (THRESHOLD*CYL_THRESH_MOD);
  seg.setInputCloud (updated);
  seg.setInputNormals (cloud_normals);

  // Obtain the cylinder inliers and coefficients
  seg.segment (*inliers_cyl, *coefficients_cylinder);

  return inliers_cyl->indices;
}
// End RANSAC Methods ---------------------------------------------------------------------------------------------

// Neighbor computation methods -----------------------------------------------------------------------------------

// Checks to see if two point clouds are neighbors
// Compares based on threshold distance (thresh)
bool areNeighbors(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB, float thresh) {
  // Loop through all points in cloudA, against all points in cloudB
  // If the distance between any two points is ever thresh or below, consider them neighbors and return true
  for(size_t i=0; i<cloudA->points.size(); i++) {
    float AX = cloudA->points[i].x;
    float AY = cloudA->points[i].y;
    float AZ = cloudA->points[i].z;
    for(size_t j=0; j<cloudB->points.size(); j++) {
      float dX = (AX - cloudB->points[j].x);
      float dY = (AY - cloudB->points[j].y);
      float dZ = (AZ - cloudB->points[j].z);
      float dist = sqrt(dX*dX+dY*dY+dZ*dZ);
      if(dist<thresh)
        return true; 
    }
  }
  return false;
}

// Takes in a vector of point clouds and returns
vector<vector<int>*> findNeighbors(vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds, float thresh) {
  if (VERBOSE) cout << "Computing neighbor map" << flush;
  vector<vector<int>*> neighborHood;
  for(int i=0; i<(int)clouds.size(); i++) {
    if (VERBOSE) cout << "." << flush;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA = clouds[i];
    // Find neighbors of cloudA (cloud at location i in vector)
    vector<int>* neighbors= new vector<int>;
    for(int j=0; j<(int)clouds.size(); j++) {
      if(i==j) continue;			
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB = clouds[j];
      if(areNeighbors(cloudA,cloudB,thresh)){
        neighbors->push_back(j);
      }
    }
    neighborHood.push_back(neighbors);
  }
  if (VERBOSE) cout << endl;
  return neighborHood;
}

// End Neighbor computation methods -------------------------------------------------------------------------------

// Update a cloud given a list of inliers
void updateCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, vector<int> inliers, 
  vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>* clouds, pcl::PointCloud<pcl::PointXYZ>::Ptr updated, 
  pcl::PointCloud<pcl::Normal>::Ptr working_normals){

  // copies all inliers of the model computed to another PointCloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *final);
  (*clouds).push_back(final);

  // Remove inliers from original cloud
  pcl::PointIndices::Ptr fInliers (new pcl::PointIndices);
  fInliers->indices= inliers;
  pcl::ExtractIndices<pcl::PointXYZ> extract; 
  extract.setInputCloud(updated); 
  extract.setIndices(fInliers);
  extract.setNegative(true); // Removes part_of_cloud from full cloud  and keep the rest
  extract.filter(*updated);

  //Remove inliers from the original normals
  pcl::ExtractIndices<pcl::Normal> extractNormals;
  extractNormals.setInputCloud(working_normals);
  extractNormals.setIndices(fInliers);
  extractNormals.setNegative(true); // Removes part_of_cloud from full cloud  and keep the rest
  extractNormals.filter(*working_normals);

  assert(updated->size() == working_normals->size() );
  
}

//Return a set of outliers given a cloud and a set of inliers
vector<int>* getOutliers(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, vector<int> inliers){
  vector<int>* outliers= new vector<int>;
  int index=0;
  for (int i=0; i<(int)(cloud->width * cloud->height); i++){
    if (i==inliers[index]){ 
      index++;
    } else {
      outliers->push_back(i);
    }
  }
  return outliers;
}

// Print the banner and usage information for this executable
void usage(){
  cout << "\nGeneralized Ransac Segmentation\n";
  cout << "  authors Caspar Anderegg and Bill Best - spring 2012\n";
  cout << "\nUsage: rosrun cfg_obj ransac [OPTIONS] <input_cloud.pcd>\n";
  cout << "\nOptions:\n";
  cout << "\n  --help		Print usage options.\n";
  cout << "  -vv		Verbose mode, print out progress during computation.\n";
  cout << "  -visual		Visual mode, display a colored segmentation of the cloud.\n";
  cout << "\n  Purpose: Uses a generalized RANSAC algorithm to segment a PCL pointcloud (.pcd) file into component shapes. Supported shapes are planes, cylinders, and spheres. Dumps the segmented .pcd file as well as a neighbor map between segments.\n" << endl;
}

//Print a shape given it's integer encoding
void printShape(int encoded){
  if (encoded==0) {
    cout << "plane, ";
  } else if (encoded==1){
    cout << "cylinder, ";
  } else if (encoded==2){
    cout << "sphere, ";
  }
}

// Main method
int main(int argc, char** argv) {

  // Parse arguments and, if necessary, print usage
  if (argc < 2){
    usage();
    exit(0);
  }
  for (int i=1; i<argc; i++){
    if (strcmp(argv[i],"-vv")==0) { // Check verbose flag
      VERBOSE= true;
    } else if (strcmp(argv[i],"-visual")==0){ // Check visual flag
      VISUAL= true;
    } else if (strcmp(argv[i],"--help")==0){ // Check help flag
      usage();
      exit(0);
    }
  }

  // initialize queue
  queue<treeNode*> toCompute;

  // initialize PointClouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr updated (new pcl::PointCloud<pcl::PointXYZ>);

  char* pcdName = argv[argc-1];
  
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcdName, *cloud) == -1) //* load the file
  {
    cout << "Couldn't read .pcd file" << endl;
    return (-1);
  }
  MAX_OUTLIERS= (int)(cloud->size() * .05); 
  if (VERBOSE) cout<<"Loaded point cloud...\n";

  // Instantiate the tree root node
  treeNode* root= new treeNode;
  root->children= new vector<treeNode*>;
  vector<int>* cloudIndices= new vector<int>;
  for (int i=0; i<(int)(cloud->width * cloud->height); i++){ cloudIndices->push_back(i); }
  root->outliers= cloudIndices;
  root->score= root->outliers->size();
  root->isRoot= true;
  if (VERBOSE) cout << "Instantiated parse tree...\n";

  vector<int> inliers;
  vector<int> outliers;
  vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;
  pcl::copyPointCloud<pcl::PointXYZ>(*cloud, *cloudIndices, *updated);
  treeNode* result=0; treeNode* workingNode;
  bool stillRunning= true;
  if (VERBOSE) cout << "Built working cloud...\n";
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ>());
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::Normal>::Ptr working_normals (new pcl::PointCloud<pcl::Normal>);

  //Compute the normals
  ne.setSearchMethod(tree);
  ne.setInputCloud(updated);
  ne.setKSearch(50);
  ne.compute(*cloud_normals);
  if (VERBOSE) cout << "Computed normals...\n";

  //Add the root treeNode to the queue
  toCompute.push(root);
  if (VERBOSE) cout<<"Fitting model";
  // Iterate over ransac models
  while (toCompute.size() > 0 && stillRunning){
    if (VERBOSE) cout << "." << flush;
    // Pop off the head of the queue
    workingNode= toCompute.front();
    toCompute.pop();

    // Build cloud of points remaining to be fit
    pcl::copyPointCloud<pcl::PointXYZ>(*cloud, *(workingNode->outliers), *updated);
    pcl::copyPointCloud<pcl::Normal>(*cloud_normals, *(workingNode->outliers), *working_normals);

    // Compute children of this node, and add them to the queue
    for (int i=0; i<SHAPES; i++){
      //Build a child node with a new shape
      treeNode* childNode= new treeNode;
      childNode->isRoot= false;
      childNode->shape= i;
      childNode->parent= workingNode;
      workingNode->children->push_back(childNode);
      childNode->children= new vector<treeNode*>;      

      //Fit a ransac shape and compute remaining outliers
      if (childNode->shape==0){
        inliers= ransacPlane(updated);
        childNode->outliers= getOutliers(updated, inliers);
        childNode->score= childNode->outliers->size();
      } else if (childNode->shape==1) {
        inliers= ransacCylinder(updated, working_normals);
        childNode->outliers= getOutliers(updated, inliers);
        childNode->score= childNode->outliers->size();
      } else if (childNode->shape==2) {
        inliers= ransacSphere(updated);
        childNode->outliers= getOutliers(updated, inliers);
        childNode->score= childNode->outliers->size();
      }

      // Push child onto queue, check score
      if (childNode->score < MAX_OUTLIERS){
        stillRunning= false;
        result= childNode;
        break;
      } else {
        toCompute.push(childNode);
      }
    }
  }
  if (VERBOSE) if (VERBOSE) cout << "\n";
  
  // Cascade back to find the path of shapes
  vector<int> reversePath;
  while (!result->isRoot){
    reversePath.push_back(result->shape);
    result= result->parent;
  }

  //Compute series of fit clouds
  pcl::copyPointCloud<pcl::PointXYZ>(*cloud, *cloudIndices, *updated);
  pcl::copyPointCloud<pcl::Normal>(*cloud_normals, *cloudIndices, *working_normals);
  cout << "Shapes Fit: ";
  for (int i=reversePath.size()-1; i>=0; i--){
    printShape(reversePath[i]);
    if (reversePath[i]==0){
      inliers= ransacPlane(updated);
    } else if (reversePath[i]==1){
      inliers= ransacCylinder(updated, working_normals);
    } else if (reversePath[i]==2){
      inliers= ransacSphere(updated);
    }
    updateCloud(updated, inliers, &clouds, updated, working_normals);
    CLUSTERS++;
  }  
  cout << "\n";

  // Print the number of iterations
  cout << "Number of clusters found: " << CLUSTERS << "\n";

  // Build segmented cloud
  if (VERBOSE) cout << "Building segmented cloud..." << endl;
  pcl::PointCloud<pcl::PointXYZRGBCamSL> segmentedPCD= *(new pcl::PointCloud<pcl::PointXYZRGBCamSL>());
  for (int i=0; i<(int)clouds.size(); i++){
    pcl::PointCloud<pcl::PointXYZRGBCamSL> segment= *(new pcl::PointCloud<pcl::PointXYZRGBCamSL>());
    segment.points.resize(clouds[i]->size());
    for (int j=0; j<(int)clouds[i]->size(); j++){
      segment.points[j].clone(clouds[i]->points[j]);
      segment.points[j].segment= i+1; // Clouds are indexed from 1
      segment.points[j].label= reversePath[reversePath.size()-1-i]; // Label contains the shape ID
    }
    segmentedPCD+= segment;
  }

  // Compute neighbor map
  vector<vector<int>*> neighborMap= findNeighbors(clouds, THRESHOLD*2.0);

  // Write segmented cloud to a file
  if (VERBOSE) cout << "Writing segments to a file..." << endl;
  string filename= string(pcdName);
  string segFile= filename.substr(0,filename.length()-4).append("_segmented.pcd");
  pcl::io::savePCDFile<pcl::PointXYZRGBCamSL>(segFile,segmentedPCD,false);

  // Write neighbor map to a file
  if (VERBOSE) cout << "Writing neighbors to a file..." << endl;
  string neighFile= filename.substr(0,filename.length()-4).append("_nbr.txt");
  std::ofstream logFile;    
  logFile.open(neighFile.data(),ios::out);
  for (size_t i = 0; i < neighborMap.size(); i++){
      logFile << i+1; // Clouds are indexed from 1
      for (size_t j = 0; j<neighborMap[i]->size(); j++){
        logFile<<","<<((*neighborMap[i])[j] + 1); // Clouds are indexed from 1
      }
      logFile << endl;
    }
    logFile.close();

  // Writing basic parse tree to a file
  if (VERBOSE) cout << "Writing base parse tree to a file...." << endl;
  string treeFile= filename.substr(0,filename.length()-4).append("_gt_tree.dot");
  logFile.open(treeFile.data(), ios::out);
  logFile << "digraph g{" << endl;
  for (size_t i=0; i<clouds.size(); i++){
    logFile << "Terminal__" << i+1 << "__";
    int ind= reversePath.size() - i - 1;
    if (reversePath[ind] == 0) logFile << "plane";
    else if (reversePath[ind] == 1) logFile << "clylinder";
    else if (reversePath[ind] == 2) logFile << "sphere";
    logFile << " ;" << endl;
  }
  logFile << "}" << endl;
  logFile.close();

  // Display segments visually
  //if (VISUAL) visualizeCloud(clouds);
  
  return 0;
}
