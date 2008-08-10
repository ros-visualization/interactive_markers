#ifndef SCENE_LABELER_H
#define SCENE_LABELER_H

#include <iostream>
#include <vector>
#include <algorithm>
#include <std_msgs/PointCloudFloat32.h>
#include <std_msgs/ImageArray.h>
#include <std_msgs/Image.h>
#include <ros/node.h>
#include <std_msgs/Empty.h>
#include <std_msgs/VisualizationMarker.h>
#include "logging/LogPlayer.h"
#include <rosTF/rosTF.h>
#include <newmat10/newmat.h>
#include <newmat10/newmatio.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "image_utils/cv_bridge.h"

#include <smartScan.h>
#include <time.h>
#include <cmath>

void SplitFilename (const string& str, string *dir, string *file)
{
  size_t found;
  found=str.find_last_of("/\\");
  *file = str.substr(found+1);
  *dir = str.substr(0,found);
}

template <class T>
void copyMsg(string name, ros::msg* m, ros::Time t, void* n)
{
  if (m != 0) {
    *((T*)(n)) = *((T*)(m));
  }
}


class scene_labeler : public ros::node
{
public:

  rosTFClient rtf;

  // -- For the full scene.
  scene_labeler(string file1) 
    : ros::node("scene_labeler"),
      rtf(*this)
  {
    advertise<std_msgs::PointCloudFloat32>("full_cloud");
    advertise<std_msgs::PointCloudFloat32>("videre/cloud");
    advertise<std_msgs::VisualizationMarker>("visualizationMarker");
    advertise<std_msgs::Image>("intensity_image");
    advertise<std_msgs::ImageArray>("labeled_images");
    advertise<std_msgs::ImageArray>("videre/images");

    lp.open(file1, ros::Time(0));
    lp.addHandler<std_msgs::ImageArray>(string("videre/images"), &copyMsg<std_msgs::ImageArray>, (void*)(&videre_images_msg_), true);
    lp.addHandler<std_msgs::ImageArray>(string("labeled_images"), &copyMsg<std_msgs::ImageArray>, (void*)(&labeled_images_msg_), true);
    lp.addHandler<std_msgs::Image>(string("intensity_image"), &copyMsg<std_msgs::Image>, (void*)(&intensity_image_msg_), true);
    lp.addHandler<std_msgs::PointCloudFloat32>(string("full_cloud"), &copyMsg<std_msgs::PointCloudFloat32>, (void*)(&full_cloud_msg_), true);
    lp.addHandler<std_msgs::PointCloudFloat32>(string("videre/cloud"), &copyMsg<std_msgs::PointCloudFloat32>, (void*)(&videre_cloud_msg_), true);
    //lp.addHandler<std_msgs::ImageArray>(string("videre/images"), &copyMsg<std_msgs::ImageArray>, (void*)(&image_msg), true);
    while(lp.nextMsg()); //Load all the messages.

    // -- Get the labeled mask out of the labeled images array.
    bridge = new CvBridge<std_msgs::Image>(&labeled_images_msg_.images[2], CvBridge<std_msgs::Image>::CORRECT_BGR | CvBridge<std_msgs::Image>::MAXDEPTH_8U);

    cout << "trying to show,," << endl;
    bridge->to_cv(&mask);
    cvNamedWindow( "Mask", CV_WINDOW_AUTOSIZE);
    cvShowImage("Mask", mask);
  }


  LogPlayer lp;
  std_msgs::ImageArray videre_images_msg_;
  std_msgs::ImageArray labeled_images_msg_;
  std_msgs::Image intensity_image_msg_;
  std_msgs::PointCloudFloat32 full_cloud_msg_;
  std_msgs::PointCloudFloat32 videre_cloud_msg_;

  CvBridge<std_msgs::Image> *bridge;
  IplImage *mask;

  LogPlayer lp2;
  std_msgs::PointCloudFloat32 cloud1, cloud2;

  void propagateLabels() {
  }

  
  void labelVidereCloud() {
    //Assumes that the transformation between videre cloud and smallv is just a rotation.  This is probably wrong.
    //Define transformation from FRAMEID_STEREO_BLOCK to FRAMEID_SMALLV
    cout << rtf.lookup("FRAMEID_STEREO_BLOCK") << endl;
    cout << rtf.lookup("FRAMEID_SMALLV") << endl;
    cout << rtf.lookup("FRAMEID_VIDERE_LEFT_IMG") << endl;
    ros::Time timenow = ros::Time::now();
    rtf.setWithEulers((unsigned int)rtf.lookup("FRAMEID_STEREO_BLOCK"), (unsigned int)rtf.lookup("FRAMEID_SMALLV"), //
		      0.0, 0.0, 0.0, -M_PI/2, -M_PI/2, 0.0, ros::Time::now().to_ull());

    // -- Project points into the smallv frame.
    cout << videre_cloud_msg_.header.frame_id << endl; //0.  wtf?
    videre_cloud_msg_.header.frame_id = rtf.lookup("FRAMEID_STEREO_BLOCK");
    videre_cloud_msg_ = rtf.transformPointCloud("FRAMEID_SMALLV", videre_cloud_msg_);    
    assert(videre_cloud_msg_.header.frame_id == rtf.lookup("FRAMEID_SMALLV"));
    publish("videre/cloud", videre_cloud_msg_);

    // -- Do the transformation to the image plane ourselves.
    NEWMAT::Matrix trns(3,4);
/*     NEWMAT::Real trnsele[] = {  7.290000e+000, 0.000000e+000, 3.239809e+000, 0.000000e+000, // */
/* 				0.000000e+000, 7.290000e+000, 2.478744e+000, 0.000000e+000, // */
/* 				0.000000e+000, 0.000000e+000, 1.000000e-002, 0.000000e+000}; */

    //Assumes the points are in mm:
    NEWMAT::Real trnsele[] = {  7.290000e+002, 0.000000e+000, 3.239809e+002, 0.000000e+000, //
				0.000000e+000, 7.290000e+002, 2.478744e+002, 0.000000e+000, //
				0.000000e+000, 0.000000e+000, 1.000000e+000, 0.000000e+000};


    trns << trnsele;
    cout << trns;
    // -- Put the videre_cloud_msg_ into a NEWMAT matrix and do the projection.
    unsigned int n = videre_cloud_msg_.get_pts_size();
    NEWMAT::Matrix vid(4, n);
    NEWMAT::Real videle[4*n];
    for (unsigned int i=0; i<n; i++) {
      videle[i] = videre_cloud_msg_.pts[i].x * 1000;
      videle[i + n] = videre_cloud_msg_.pts[i].y  * 1000;
      videle[i + 2*n] = videre_cloud_msg_.pts[i].z * 1000;
      videle[i + 3*n] = 1;
    }
    vid << videle;
    NEWMAT::Matrix projected = trns * vid;

    // -- Normalize so z = 1.
    for (int i=1; i<=n; i++) {
      projected(1,i) = projected(1,i) / projected(3,i);
      projected(2,i) = projected(2,i) / projected(3,i);
      projected(3,i) = 1;
    } 

    cout << endl << "Max x: " << projected.Row(1).Maximum() << "   Min x: " << projected.Row(1).Minimum() << endl;
    cout << endl << "Max y: " << projected.Row(2).Maximum() << "   Min y: " << projected.Row(2).Minimum() << endl;
    cout << endl << "Max z: " << projected.Row(3).Maximum() << "   Min z: " << projected.Row(3).Minimum() << endl;

    // -- Put the points back into a message and publish to see what it looks like.
    for (unsigned int i=1; i<=n; i++) {
      //cout << i << endl;
      videre_cloud_msg_.pts[i].x = projected(1, i) / 1000;
      videre_cloud_msg_.pts[i].y = projected(2, i) / 1000;
      videre_cloud_msg_.pts[i].z = projected(3, i) / 1000;
      videre_cloud_msg_.chan[0].vals[i] = 255;
      cout << cvGet2D(mask,floor(projected(1, i)+.5), floor(projected(2,i)+.5)).val[0] << " ";
    }
    usleep(3000000);
    publish("videre/cloud", videre_cloud_msg_);
    


    // -- Get the labels into the cloud.
    // -- Make a modified pointcloud with intensities showing the labels for debugging.
    // -- Publish the labeled pointcloud for debugging.
  }

  void publishAll() {
    publish("full_cloud", full_cloud_msg_);
    publish("labeled_images", labeled_images_msg_);
    publish("videre/images", videre_images_msg_);
    publish("intensity_image", intensity_image_msg_);
    publish("videre/cloud", videre_cloud_msg_);
  }

};


#endif
