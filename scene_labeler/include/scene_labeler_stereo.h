/**
@mainpage

@htmlinclude manifest.html

@b scene_labeler is a package of tools for collecting and using labeled training examples of images + spacetime stereo pointclouds for machine learning tasks.  Its main uses are:
- Including a SceneLabelerStereo object with code that needs access to the data inside .bag files stored by guide_stereo.
- Collecting labeled data with data_collection.xml.
- Running it as a command line tool to play back labeled point clouds for inspection of data.

@section usage Usage

To play back logs collected by data_collection.xml:
@verbatim
$ scene_labeler_stereo BAGFILE [BAGFILE ...]
@endverbatim

Publishes to (name / type):
- @b videre/images / ImageArray
- @b spacetime_stereo  / PointCloud
- @b visualizationMarker / VisualizationMarker
- @b labeled_images / ImageArray

To collect new data:
@verbatim
$ roslaunch data_collection.xml
@endverbatim

There is a wiki page here addressing the details of data collection:
http://pr.willowgarage.com/wiki/DataCollection

**/

#ifndef SCENE_LABELER_STEREO_H
#define SCENE_LABELER_STEREO_H

#include <iostream>
#include <vector>
#include <algorithm>
#include <std_msgs/PointCloud.h>
#include <std_msgs/ImageArray.h>
#include <std_msgs/Image.h>
#include <std_msgs/String.h>
#include <ros/node.h>
#include <std_msgs/Empty.h>
#include <std_msgs/VisualizationMarker.h>
#include "rosrecord/Player.h"
#include <newmat10/newmat.h>
#include <newmat10/newmatio.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "opencv/cxcore.h"
#include "image_utils/cv_bridge.h"

#include <smartScan.h>
#include <time.h>
#include <cmath>

template <class T>
void copyMsg(string name, ros::msg* m, ros::Time t, void* n)
{
  if (m != 0) {
    *((T*)(n)) = *((T*)(m));
  }
  cout << "copying " << name << endl;
}

class SceneLabelerStereo
{
public:
  //! ROS peer usage style: this node will be used as the object's communication device.
  ros::node* node_;
  std_msgs::ImageArray images_msg_;
  std_msgs::ImageArray labeled_images_msg_;
  std_msgs::Image intensity_image_msg_;
  std_msgs::PointCloud cloud_;
  std_msgs::String cal_params_msg_;
  //! Image with label mask.
  IplImage *mask_;
  //! Left camera image.
  IplImage *left_;
  //! Right camera image.
  IplImage *right_;
  //! The SmartScan that contains all the points in the scene.
  SmartScan ss_cloud_;
  //! Contains separate SmartScans of each label.
  map<int, SmartScan> ss_labels_;
  //! Contains SmartScans of every object paired with the object's label.
  vector< pair<int, SmartScan*> > ss_objs_;
  //! Cross-indexing of pointcloud and image.  xidx_[row][col] returns the point id of the corresponding point in ss_cloud_.
  vector< vector<int> > xidx_;
  //! "spacetime_stereo" by default.
  string ptcld_topic_;

 SceneLabelerStereo(ros::node* node) 
   : node_(node), mask_(0), left_(0), right_(0), objects_extracted_(false)
  {
    ptcld_topic_ = string("spacetime_stereo");
    loaded_msgs_ = false;
    trns_ = NEWMAT::Matrix(3,4); trns_ = 0.0;
    srand(time(NULL));
  }

  ~SceneLabelerStereo() 
    {
      if(objects_extracted_) {
	cvReleaseImage(&mask_);
	cvReleaseImage(&right_);
	cvReleaseImage(&left_);
	for(unsigned int i=0; i<ss_objs_.size(); i++) {
	  delete ss_objs_[i].second;
	}
	delete bridge_mask_;
	delete bridge_right_;
	delete bridge_left_;
      }
    }

  //! Load a bag file saved with data_collection.xml.
  void loadMsgsFromFile(string file);
  //! Put the relevant messages into the SceneLabelerStereo directly, rather than loading from a file.  This is used, for example, when you want to get a image - pointcloud crossindexing but don't have a label mask.
  void loadMsgsFromMem(std_msgs::ImageArray images_msg, std_msgs::PointCloud cloud, std_msgs::String cal_params_msg);
  //! Label and extract the objects from the scene.
  void processMsgs();
  //! Publishes all the data of the scene, but with the point cloud colored with the labeling from the label mask.
  void publishAll();
  void getRandomPointFromPointcloud(float *x, float *y, float *z, int *row, int *col, SmartScan *ss = NULL);
  bool getRandomPointFromImage(float *x, float *y, float *z, int *row, int *col, int label=-1);
  //! Project points in the point cloud into the image plane and generate the cross indexing.  This is normally done as part of labeling the point cloud, but also is useful for generating a cross indexing of a pointcloud to an image without needing a label mask. (i.e. for classification).
  void projectAndCrossIndex(); 
  //! Adds another channel to cloud_ that contains the labels.
  void labelCloud();
  //! Given a point in the point cloud, find the corresponding point in the image.
  void getImageIdx(float x, float y, float z, float *row, float *col);
  //! Sets ss_objs_ and ss_labels_ given a labeled point cloud.
  void extractObjectsFromCloud();

 private:
  ros::record::Player lp;
  CvBridge<std_msgs::Image> *bridge_mask_, *bridge_left_, *bridge_right_;  
  //! The 3d points projected into the image.
  NEWMAT::Matrix projected_; 
  //! Transformation matrix to project points from the pointcloud into a pixel of the image.
  NEWMAT::Matrix trns_;
  bool objects_extracted_;
  bool loaded_msgs_;
  std_msgs::PointCloud cloud_colored_;

  std_msgs::PointCloud colorPointCloud(std_msgs::PointCloud ptcld);
};

#endif
