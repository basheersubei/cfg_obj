// Title: ransac.cpp
// Authors: Caspar Anderegg and Bill Best - Spring 2012
// Purpose: To implement a generalized multi-shape ransac algorithm for segmenting PCL point clouds

// Include the necessary standard libraries
#include <iostream>
#include <cstdlib>
# include <queue>
// Include necessary ros/pcl libraries
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <boost/thread/thread.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

// Using declarations
using std::cout;
using std::vector;
using std::queue;
using std::flush;

// Set constants
int MAX_OUTLIERS;
const float PERCENT_OUTLIERS= .025;
const float THRESHOLD= 0.0025;
const float CYL_THRESH_MOD= 5;
int CLUSTERS= 0;
const int SHAPES= 3;
const int MAX_CYL_ITERATIONS= 250;

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
}

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

// Compute the best ransac fit for a single plane
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
  // initialize queue
  queue<treeNode*> toCompute;

  // initialize PointClouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr updated (new pcl::PointCloud<pcl::PointXYZ>);

  char* pcdName = argv[1];
  
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcdName, *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read .pcd file\n");
    return (-1);
  }
  MAX_OUTLIERS= (int)(cloud->size() * .05); 
  cout<<"Loaded point cloud...\n";

  // Instantiate the tree root node
  treeNode* root= new treeNode;
  root->children= new vector<treeNode*>;
  vector<int>* cloudIndices= new vector<int>;
  for (int i=0; i<(int)(cloud->width * cloud->height); i++){ cloudIndices->push_back(i); }
  root->outliers= cloudIndices;
  root->score= root->outliers->size();
  root->isRoot= true;
  cout << "Instantiated parse tree...\n";

  vector<int> inliers;
  vector<int> outliers;
  vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;
  pcl::copyPointCloud<pcl::PointXYZ>(*cloud, *cloudIndices, *updated);
  treeNode* result=0; treeNode* workingNode;
  bool stillRunning= true;
  cout << "Built working cloud...\n";
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ>());
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::Normal>::Ptr working_normals (new pcl::PointCloud<pcl::Normal>);

  //Compute the normals
  ne.setSearchMethod(tree);
  ne.setInputCloud(updated);
  ne.setKSearch(50);
  ne.compute(*cloud_normals);
  cout << "Computed normals...\n";

  //Add the root treeNode to the queue
  toCompute.push(root);
  cout<<"Fitting model";
  // Iterate over ransac models
  while (toCompute.size() > 0 && stillRunning){
    cout << "." << flush;
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
  cout << "\n";
  
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
  visualizeCloud(clouds);

  return 0;
 }
