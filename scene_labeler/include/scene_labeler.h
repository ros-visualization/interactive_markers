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
#include "opencv/cxcore.h"
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

  scene_labeler(string file1) 
    : ros::node("scene_labeler"),
    rtf(*this),
    mask(0)
  {
    advertise<std_msgs::PointCloudFloat32>("full_cloud");
    advertise<std_msgs::PointCloudFloat32>("videre/cloud");
    advertise<std_msgs::VisualizationMarker>("visualizationMarker");
    advertise<std_msgs::Image>("intensity_image");
    advertise<std_msgs::ImageArray>("labeled_images");
    advertise<std_msgs::ImageArray>("videre/images");


    cout << "Loading messages... ";
    lp.open(file1, ros::Time(0));
    lp.addHandler<std_msgs::ImageArray>(string("videre/images"), &copyMsg<std_msgs::ImageArray>, (void*)(&videre_images_msg_), true);
    lp.addHandler<std_msgs::ImageArray>(string("labeled_images"), &copyMsg<std_msgs::ImageArray>, (void*)(&labeled_images_msg_), true);
    lp.addHandler<std_msgs::Image>(string("intensity_image"), &copyMsg<std_msgs::Image>, (void*)(&intensity_image_msg_), true);
    lp.addHandler<std_msgs::PointCloudFloat32>(string("full_cloud"), &copyMsg<std_msgs::PointCloudFloat32>, (void*)(&full_cloud_msg_), true);
    lp.addHandler<std_msgs::PointCloudFloat32>(string("videre/cloud"), &copyMsg<std_msgs::PointCloudFloat32>, (void*)(&videre_cloud_msg_), true);
    while(lp.nextMsg()); //Load all the messages.
    cout << "Done." << endl;
    publish("videre/cloud", videre_cloud_msg_);
    usleep(1000000);
    // -- Get the labeled mask out of the labeled images array.
    cout << "Making new bridge...";
    bridge = new CvBridge<std_msgs::Image>(&labeled_images_msg_.images[2], CvBridge<std_msgs::Image>::CORRECT_BGR | CvBridge<std_msgs::Image>::MAXDEPTH_8U);
    assert(bridge->to_cv(&mask));
    cout << "Done." << endl;

    // -- Get transformations over ros.
    cout << "Waiting for transformations from frameServer... "; flush(cout);
    while(true) {
      try {
	rtf.getMatrix(rtf.lookup("FRAMEID_SMALLV"), rtf.lookup("FRAMEID_TILT_BASE"), ros::Time::now().to_ull());
	break;
      }
      catch (libTF::TransformReference::LookupException & ex) {
	usleep(100000);
      }
    }
    cout << "Done." << endl; 

    // -- Apply a default frame for old logs that don't have it in the header.
    if(videre_cloud_msg_.header.frame_id == 0)
      videre_cloud_msg_.header.frame_id = rtf.lookup("FRAMEID_STEREO_BLOCK");
    if(full_cloud_msg_.header.frame_id == 0)
      full_cloud_msg_.header.frame_id = rtf.lookup("FRAMEID_TILT_BASE");
  
    // -- Old time stamps anger rostf.
    full_cloud_msg_.header.stamp = ros::Time::now();
    videre_cloud_msg_.header.stamp = ros::Time::now();
    
    // -- Propagate the labels.
    cout << "labeling videre with frame id " << videre_cloud_msg_.header.frame_id << endl;
    labelCloud(&videre_cloud_msg_);
    cout << "labeling hokuyo with frame id " << full_cloud_msg_.header.frame_id << endl;
    labelCloud(&full_cloud_msg_);

  }

  rosTFClient rtf;

  LogPlayer lp;
  std_msgs::ImageArray videre_images_msg_;
  std_msgs::ImageArray labeled_images_msg_;
  std_msgs::Image intensity_image_msg_;
  std_msgs::PointCloudFloat32 full_cloud_msg_;
  std_msgs::PointCloudFloat32 videre_cloud_msg_;

  std_msgs::PointCloudFloat32 full_cloud_labeled_;
  std_msgs::PointCloudFloat32 full_cloud_colored_;
  std_msgs::PointCloudFloat32 videre_cloud_colored_;

  CvBridge<std_msgs::Image> *bridge;
  IplImage *mask;


  
  void labelCloud(std_msgs::PointCloudFloat32 *ptcld) {
 
    //Define transformation from FRAMEID_STEREO_BLOCK to FRAMEID_SMALLV
/*     cout << rtf.lookup("FRAMEID_STEREO_BLOCK") << endl; */
/*     cout << rtf.lookup("FRAMEID_SMALLV") << endl; */
/*     cout << rtf.lookup("FRAMEID_TILT_BASE") << endl; */
/*     cout << rtf.lookup("FRAMEID_VIDERE_LEFT_IMG") << endl;     */
/*     cout << rtf.viewFrames() << endl; */

    // -- Debugging information.
/*     cout << "transform from tilt to smallv" << endl; */
/*     cout << rtf.getMatrix(rtf.lookup("FRAMEID_SMALLV"), rtf.lookup("FRAMEID_TILT_BASE"), ros::Time::now().to_ull()) << endl; */
/*     cout << "transform from stereo block to smallv" << endl; */
/*     cout << rtf.getMatrix(rtf.lookup("FRAMEID_SMALLV"), rtf.lookup("FRAMEID_STEREO_BLOCK"), ros::Time::now().to_ull()) << endl; */
/*     NEWMAT::Matrix id =       rtf.getMatrix(rtf.lookup("FRAMEID_SMALLV"), rtf.lookup("FRAMEID_STEREO_BLOCK"), ros::Time::now().to_ull()) * // */
/*       rtf.getMatrix(rtf.lookup("FRAMEID_STEREO_BLOCK"), rtf.lookup("FRAMEID_TILT_BASE"), ros::Time::now().to_ull()); */
/*     cout << "tilt to smallv" << endl << id << endl; */
/*     cout << "cloud frameid is " << ptcld->header.frame_id << endl; //0.  wtf? */

    
    try {
      *ptcld = rtf.transformPointCloud("FRAMEID_SMALLV", *ptcld);
    }
    catch  (libTF::TransformReference::LookupException & ex) {
      cout << "failure to find parent of frame id" << endl;
      cout << "frame id is "  << ptcld->header.frame_id << endl;
      cout << rtf.viewFrames() << endl;
      return;
    }
      

    // -- Set up smallv to image plane transformation.
    //Assumes the points are in mm:
    NEWMAT::Real trnsele[] = {  7.290000e+002, 0.000000e+000, 3.239809e+002, 0.000000e+000, //
				0.000000e+000, 7.290000e+002, 2.478744e+002, 0.000000e+000, //
				0.000000e+000, 0.000000e+000, 1.000000e+000, 0.000000e+000};
    NEWMAT::Matrix trns(3,4);
    trns << trnsele;

    // -- Put the videre_cloud_msg_ into a NEWMAT matrix and do the projection.
    unsigned int n = ptcld->get_pts_size();
    NEWMAT::Matrix vid(4, n);
    NEWMAT::Real videle[4*n];
    for (unsigned int i=0; i<n; i++) {
      videle[i] = ptcld->pts[i].x * 1000; //in millimeters
      videle[i + n] = ptcld->pts[i].y  * 1000;
      videle[i + 2*n] = ptcld->pts[i].z * 1000;
      videle[i + 3*n] = 1;
    }
    vid << videle;
    NEWMAT::Matrix projected = trns * vid;

    // -- Normalize so z = 1.
    for ( unsigned int i=1; i<=n; i++) {
      projected(1,i) = projected(1,i) / projected(3,i);
      projected(2,i) = projected(2,i) / projected(3,i);
      projected(3,i) = 1;
    } 

    /* cout << endl << "Max x: " << projected.Row(1).Maximum() << "   Min x: " << projected.Row(1).Minimum() << endl; */
/*     cout << endl << "Max y: " << projected.Row(2).Maximum() << "   Min y: " << projected.Row(2).Minimum() << endl; */
/*     cout << endl << "Max z: " << projected.Row(3).Maximum() << "   Min z: " << projected.Row(3).Minimum() << endl; */

    // -- Add the channel with the labeling.
    std_msgs::PointCloudFloat32 tmp;
    tmp.set_pts_size(n);
    tmp.set_chan_size(2);
    tmp.chan[0] = ptcld->chan[0];
    tmp.header = ptcld->header;
    //tmp.chan[0].set_vals_size(n);
    tmp.chan[1].set_vals_size(n);
    //tmp.chan[0].name = "intensities";
    tmp.chan[1].name = "labels";
    cout << "hxw " << mask->height << " " << mask->width << endl;
    for (unsigned int i=1; i<=n; i++) {
      tmp.pts[i-1] = ptcld->pts[i-1];
      int c = floor(projected(1, i)+.5);
      int r = floor(projected(2, i)+.5);
      if(r >= 0 && r < mask->height && //
	 c >= 0 && c < mask->width && //
	 cvGet2D(mask, r, c).val[0] != 0)
	tmp.chan[1].vals[i-1] = cvGet2D(mask, r, c).val[0];
      else
        tmp.chan[1].vals[i-1] = 0;
    }

    ptcld->set_chan_size(2);
    *ptcld = tmp;
    cout << "label: chan size is " << ptcld->get_chan_size() << " and npts is " << ptcld->get_pts_size() << endl;
  }

  std_msgs::PointCloudFloat32 colorPointCloud(std_msgs::PointCloudFloat32 ptcld) {
    cout << "chan size is " << ptcld.get_chan_size() << endl;
    for (unsigned int i=0; i<ptcld.get_pts_size(); i++) {
      if(ptcld.chan[1].vals[i] != 0) 
	ptcld.chan[0].vals[i] = 255;
      else
	ptcld.chan[0].vals[i] = 100;
    }
    ptcld.chan[0].vals[0] = 0; //Because ogre vis scales the colors.

    return ptcld;

  }


  void publishAll() {


/*     // -- Put the points from the cloud into the mask image. */
/*     for (unsigned int i=0; i<n; i++) { */
/*       int c = floor(projected(1, i+1)+.5); */
/*       int r = floor(projected(2, i+1)+.5); */
/*       if(r >= 0 && r < mask->height && // */
/* 	 c >= 0 && c < mask->width) { */
/* 	CvScalar s=cvGet2D(mask,r,c); */
/* 	s.val[0] = 100; */
/* 	cvSet2D(mask, r, c, s); */
/*       } */
/*     } */
/*     cvNamedWindow( "mask", CV_WINDOW_AUTOSIZE); */
/*     cvShowImage("mask", mask); */
/*     cout << " waiting for keypress..." << endl; */
/*     cvWaitKey(); */

    // -- Display the mask for debugging.
/*     cout << "trying to show,," << endl; */
/*     cvNamedWindow( "mask", CV_WINDOW_AUTOSIZE); */
/*     //mask=cvLoadImage("/u/teichman/data/08-08-08_calib2/000-L.png"); */
/*     if(!mask) */
/*       cout << "couldn't load" << endl; */
/*     cvShowImage("mask", mask); */
/*     cout << " waiting for keypress..." << endl; */
/*     cvWaitKey(); */

    cout << "Publishing... " << endl;
    full_cloud_colored_ = colorPointCloud(full_cloud_msg_);
    publish("full_cloud", full_cloud_colored_);
    //usleep(1000000);
    videre_cloud_colored_ = colorPointCloud(videre_cloud_msg_); 
    publish("videre/cloud", videre_cloud_msg_);
    //usleep(1000000);
    publish("labeled_images", labeled_images_msg_);
    publish("videre/images", videre_images_msg_);
    publish("intensity_image", intensity_image_msg_);
    publish("videre/cloud", videre_cloud_colored_);
  }
};
#endif
