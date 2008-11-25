#include <stdio.h>
#include <scene_labeler_stereo.h>
#include <limits>

using namespace std;


void SceneLabelerStereo::loadMsgsFromFile(string file) {    
  node_->advertise<std_msgs::PointCloud>(ptcld_topic_, 100);
  node_->advertise<std_msgs::VisualizationMarker>("visualizationMarker", 100);
  node_->advertise<std_msgs::ImageArray>("labeled_images", 100);
  node_->advertise<std_msgs::ImageArray>("videre/images", 100);


  cout << "Loading messages... "; flush(cout);
  lp.open(file, ros::Time(0.0));
  lp.addHandler<std_msgs::ImageArray>(string("videre/images"), &copyMsg<std_msgs::ImageArray>, (void*)(&images_msg_), true);
  lp.addHandler<std_msgs::ImageArray>(string("labeled_images"), &copyMsg<std_msgs::ImageArray>, (void*)(&labeled_images_msg_), true);
  lp.addHandler<std_msgs::PointCloud>(ptcld_topic_, &copyMsg<std_msgs::PointCloud>, (void*)(&cloud_), true);
  lp.addHandler<std_msgs::String>(string("videre/cal_params"), &copyMsg<std_msgs::String>, (void*)(&cal_params_msg_), true);
  while(lp.nextMsg()); //Load all the messages.
  lp.close();
  cout << "Done." << endl;
  loaded_msgs_ = true;

    
  // -- Get the labeled mask out of the labeled images array.
  cout << "Extracting images...";
  bridge_mask_ = new CvBridge<std_msgs::Image>(&labeled_images_msg_.images[2], CvBridge<std_msgs::Image>::CORRECT_BGR | CvBridge<std_msgs::Image>::MAXDEPTH_8U);
  assert(bridge_mask_->to_cv(&mask_));

  bridge_left_ = new CvBridge<std_msgs::Image>(&labeled_images_msg_.images[1], CvBridge<std_msgs::Image>::CORRECT_BGR | CvBridge<std_msgs::Image>::MAXDEPTH_8U);
  assert(bridge_left_->to_cv(&left_));

  bridge_right_ = new CvBridge<std_msgs::Image>(&images_msg_.images[0], CvBridge<std_msgs::Image>::CORRECT_BGR | CvBridge<std_msgs::Image>::MAXDEPTH_8U);
  assert(bridge_right_->to_cv(&right_));
  cout << "Done." << endl;
}


void SceneLabelerStereo::loadMsgsFromMem(std_msgs::ImageArray images_msg, std_msgs::PointCloud cloud, std_msgs::String cal_params_msg) {
  xidx_.clear();
  cal_params_msg_ = cal_params_msg;
  cloud_ = cloud;
  images_msg_ = images_msg;

    
  // -- Get the labeled mask out of the labeled images array.
  cout << "Extracting images...";
  
  bridge_left_ = new CvBridge<std_msgs::Image>(&images_msg_.images[1], CvBridge<std_msgs::Image>::CORRECT_BGR | CvBridge<std_msgs::Image>::MAXDEPTH_8U);
  assert(bridge_left_->to_cv(&left_));

  bridge_right_ = new CvBridge<std_msgs::Image>(&images_msg_.images[0], CvBridge<std_msgs::Image>::CORRECT_BGR | CvBridge<std_msgs::Image>::MAXDEPTH_8U);
  assert(bridge_right_->to_cv(&right_));
  cout << "Done." << endl;
}

void SceneLabelerStereo::processMsgs() {
  assert(loaded_msgs_);

  cout << labeled_images_msg_.images[2].label << endl; 
  //cout << "Image: " << labeled_images_msg_.images[2].compression << " ";
		       

  // -- Old time stamps anger rostf.
  cloud_.header.stamp = ros::Time::now();
    
  // -- Propagate the labels.
  //cout << "labeling videre with frame id " << cloud_.header.frame_id << endl;
  labelCloud();
  extractObjectsFromCloud();
}

void SceneLabelerStereo::publishAll() {
  // -- Publish the messages.
  cout << "  Publishing scene messages... " << endl;
  cloud_colored_ = colorPointCloud(cloud_); 
  node_->publish(ptcld_topic_, cloud_colored_);
  node_->publish("labeled_images", labeled_images_msg_);
  node_->publish("videre/images", images_msg_);
  cout << "  Press Enter to see individual objects . . .";
  cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

  // -- Show the objects and their labels.
  cout << "  Showing objects..." << endl;
  for(unsigned int i=0; i<ss_objs_.size(); i++) {
    std_msgs::PointCloud debug = ss_objs_[i].second->getPointCloud();
    debug.header.frame_id = "FRAMEID_SMALLV";
    node_->publish(ptcld_topic_, debug);
    cout << "  Published object " << i << " with class label " << ss_objs_[i].first << ". Press Enter to continue . . .";
    cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }
}

void SceneLabelerStereo::getRandomPointFromPointcloud(float *x, float *y, float *z, int *row, int *col, SmartScan *ss) {
  if(!objects_extracted_) {
    cerr << "Don't call getRandomPoint before extracting objects!" << endl;
  }

  if(!ss) {
    ss = &ss_cloud_;
  }

  int randId = 0;
  randId = rand() % ss->size();
  std_msgs::Point32 pt = ss->getPoint(randId);
  *x = pt.x;
  *y = pt.y;
  *z = pt.z;

  NEWMAT::Matrix point(4,1);
  point(1,1) = pt.x;
  point(2,1) = pt.y;
  point(3,1) = pt.z;
  point(4,1) = 1;
  NEWMAT::Matrix projected = trns_ * point;
  projected(1,1) = projected(1,1) / projected(3,1);
  projected(2,1) = projected(2,1) / projected(3,1);
  projected(3,1) = 1;

  *row = (int)projected(2,1);
  *col = (int)projected(1,1);
  /*     cout << "sls: " << projected(1,1) << " being rounded to " << *row << endl; */
  /*     cout << "sls: " << projected(2,1) << " being rounded to " << *col << endl; */
}

bool SceneLabelerStereo::getRandomPointFromImage(float *x, float *y, float *z, int *row, int *col, int label) {

  while(true) {
    *col = rand() % mask_->width;
    *row = rand() % mask_->height;

    CvScalar s = cvGet2D(mask_, *row, *col);
    //int val = CV_IMAGE_ELEM(mask_, uchar, *row, *col);

    //cout << "check: " << val << " " << s.val[0] << endl;
    //assert(val == s.val[0]);

    cout << s.val[0] << " at " << *row << " " << *col << endl;
    if(s.val[0] == label | label == -1)
      break;
  }


  if(!objects_extracted_) {
    cerr << "Don't call getRandomPoint before extracting objects!" << endl;
    return false;
  }

  int ptId = xidx_[*row][*col];
  if(ptId==-1) {
    return false;
  }

  std_msgs::Point32 pt = cloud_.pts[ptId];
  *x = pt.x;
  *y = pt.y;
  *z = pt.z;

  return true;
}

void SceneLabelerStereo::projectAndCrossIndex() {
           
  // -- Set up smallv to image plane transformation.
  //Assumes the points are in mm:
  string cal = cal_params_msg_.data;
  string proj = cal.substr(cal.find("proj"), cal.find("rect"));
  /*     int position = cal.find("dpx"); */
  /*     string substring = cal.substr(position, cal.find("alpha")-position); */
  /*     float  trash, trash2, u0, v0, fx, fy; */
  /*     sscanf(substring.c_str(), "%*s %f %*s %f %*s %*f %*s %f %*s %f %*s %f %*s %f", &trash, &trash2, &u0, &v0, &fx, &fy);     */
  /*     trns_(1,1) = fx; */
  /*     trns_(1,3) = u0; */
  /*     trns_(2,2) = fy; */
  /*     trns_(2,3) = v0; */
  /*     trns_(3,3) = 1; */
  /*     cout << trns_; */

  NEWMAT::Real trnsele[12];
  // -- Put the cloud_ into a NEWMAT matrix and do the projection.
  //This is terrible.
  sscanf(proj.c_str(), "%*s %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %*s", &trnsele[0], &trnsele[1], &trnsele[2], &trnsele[3], &trnsele[4], &trnsele[5], &trnsele[6], &trnsele[7], &trnsele[8], &trnsele[9], &trnsele[10], &trnsele[11]);
  trns_ << trnsele;
  cout << "trns: " << endl << trns_ << endl;

  // -- Put the cloud_ into a NEWMAT matrix and do the projection.

  /*     unsigned int n = cloud_->get_pts_size(); */
  /*     NEWMAT::Matrix projected_(3,n); */
  /*     cout << n << " points." << endl; */

  /*     NEWMAT::Matrix vid(4, 1);  */
  /*     for (unsigned int i=0; i<n; i++) { */
  /*       //cout << i << endl; */
  /*       vid(1,1) = cloud_->pts[i].x * 1000; //in millimeters */
  /*       vid(2,1) = cloud_->pts[i].y * 1000; //in millimeters */
  /*       vid(3,1) = cloud_->pts[i].z * 1000; //in millimeters */
  /*       vid(4,1) = 1; */
  /*       projected_.Column(i+1) = trns_ * vid; */
  /*       //cout << projected_.Column(i+1) << endl; */
  /*     } */


  // -- videle gets too big and segfaults here sometimes!  The above might be better but is untested.
  unsigned int n = cloud_.get_pts_size();
  cout << n << " points." << endl;
  NEWMAT::Matrix vid(4, n);
  NEWMAT::Real videle[4*n];
  for (unsigned int i=0; i<n; i++) {
    videle[i] = cloud_.pts[i].x * 1000; //in millimeters
    videle[i + n] = cloud_.pts[i].y  * 1000;
    videle[i + 2*n] = cloud_.pts[i].z * 1000;
    videle[i + 3*n] = 1;
  }
  vid << videle;
  projected_ = trns_ * vid;

  // -- Normalize so z = 1.  Make the pixel / point cross-indexing for later while we're at it.
  for(int i=0; i<left_->height; i++) {
    vector<int> tmp(left_->width, -1);
    xidx_.push_back(tmp);
  }
    
  for ( unsigned int i=1; i<=n; i++) {
    //cout << "*** " << projected_(1,i) << " " << projected_(2,i) << " " << projected_(3,i) << endl;
    assert(projected_(3,i) != 0);
    projected_(1,i) = projected_(1,i) / projected_(3,i);
    projected_(2,i) = projected_(2,i) / projected_(3,i);
    projected_(3,i) = 1;
    //cout << projected_.Column(i);
    int row = (int)projected_(2,i);
    int col = (int)projected_(1,i);
    if(row >= 480 || row < 0 || col >= 640 || col < 0) {
      cout << "row: " << (int)projected_(2,i) << " col: " << (int)projected_(1,i) << endl;
      //cerr << "Was this pointcloud calibrated correctly?  This doesn't look right?" << endl;
      //exit(0);
    }
    else {
      xidx_[(int)projected_(2,i)][(int)projected_(1,i)] = i-1; //xidx_[row][col]
    }
  }
}

void SceneLabelerStereo::labelCloud() {
  projectAndCrossIndex();

  /* cout << endl << "Max x: " << projected_.Row(1).Maximum() << "   Min x: " << projected_.Row(1).Minimum() << endl; */
  /*     cout << endl << "Max y: " << projected_.Row(2).Maximum() << "   Min y: " << projected_.Row(2).Minimum() << endl; */
  /*     cout << endl << "Max z: " << projected_.Row(3).Maximum() << "   Min z: " << projected_.Row(3).Minimum() << endl; */

  // -- Add the channel with the labeling.
  std_msgs::PointCloud tmp;
  unsigned int n = cloud_.get_pts_size();
  tmp.set_pts_size(n);
  tmp.set_chan_size(2);
  //cout << "copying " << cloud_.chan[0].name.data << endl;
  tmp.chan[0] = cloud_.chan[0];
  tmp.header = cloud_.header;
  //tmp.chan[0].set_vals_size(n);
  tmp.chan[1].set_vals_size(n);
  //tmp.chan[0].name = "intensities";
  tmp.chan[1].name = "labels";
  cout << "hxw " << mask_->height << " " << mask_->width << endl;
  for (unsigned int i=1; i<=n; i++) {
    tmp.pts[i-1] = cloud_.pts[i-1];
    int c = floor(projected_(1, i)+.5);
    int r = floor(projected_(2, i)+.5);
    if(r >= 0 && r < mask_->height && //
       c >= 0 && c < mask_->width && //
       cvGet2D(mask_, r, c).val[0] != 0)
      tmp.chan[1].vals[i-1] = cvGet2D(mask_, r, c).val[0];
    else
      tmp.chan[1].vals[i-1] = 0;
  }

  cloud_.set_chan_size(2);
  cloud_ = tmp;
  cout << "label: chan size is " << cloud_.get_chan_size() << " and npts is " << cloud_.get_pts_size() << endl;
}

std_msgs::PointCloud SceneLabelerStereo::colorPointCloud(std_msgs::PointCloud ptcld) {
  if(ptcld.get_chan_size() == 1) {
    cout << "chan size is " << ptcld.get_chan_size() << endl;
    cout << "Not coloring..." << endl;
    return ptcld;
  }

  for (unsigned int i=0; i<ptcld.get_pts_size(); i++) {
    if(ptcld.chan[1].vals[i] != 0) 
      ptcld.chan[0].vals[i] = 255;
    else
      ptcld.chan[0].vals[i] = 100;
  }
  ptcld.chan[0].vals[0] = 0; //Because ogre vis scales the colors.

  return ptcld;

}

void SceneLabelerStereo::getImageIdx(float x, float y, float z, float *row, float*col) {
  NEWMAT::Matrix point(3,1);
  point(1,1) = x;
  point(2,1) = y;
  point(3,1) = z;
  NEWMAT::Matrix projected = trns_ * point;
  projected(1,1) = projected(1,1) / projected(3,1);
  projected(2,1) = projected(2,1) / projected(3,1);
  projected(3,1) = 1;
  *row = projected(1,1);
  *col = projected(2,1);
}

void SceneLabelerStereo::extractObjectsFromCloud() {
  map<int, int> nPts_for_label; //map<label, npts>
  map<int, int>::iterator it;
  std_msgs::PointCloud debug;

  ss_cloud_.setFromRosCloud(cloud_);

  // -- Find nPts of each label.
  for(unsigned int i=0; i<cloud_.get_pts_size(); i++) {
    int lbl = cloud_.chan[1].vals[i];
    /*       if(lbl == 0) */
    /* 	continue; */

    if(nPts_for_label.find(lbl) == nPts_for_label.end()) {
      nPts_for_label[lbl] = 0;
    }
    nPts_for_label[lbl]++;
  }

  // -- For each label, make a SmartScan.
  for(it = nPts_for_label.begin(); it != nPts_for_label.end(); it++) {
    cout << "label " << it->first << " has " << it->second << " pts. " << endl;
    float *pts = new float[it->second * 3];
    int ptsctr = 0;
    for(unsigned int i=0; i<cloud_.get_pts_size(); i++) {
      if(cloud_.chan[1].vals[i] == it->first) {
	pts[ptsctr++] = cloud_.pts[i].x;
	pts[ptsctr++] = cloud_.pts[i].y;
	pts[ptsctr++] = cloud_.pts[i].z;
      }
    }

    ss_labels_[it->first].setPoints(it->second, pts);
    /*       debug = ss_labels_[it->first].getPointCloud(); */
    /*       node_->publish("videre/cloud", debug); */
    /*       cout << "Published cloud. Press Enter to continue . . .\n"; */
    /*       cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); */
    delete[] pts; pts = NULL;
  }

  // -- For each ss_labels_, get connected components and clean up the cloud.
  map<int, SmartScan>::iterator itss;
  for(itss= ss_labels_.begin(); itss != ss_labels_.end(); itss++) {

    //Ignore the background class.
    if(itss->first == 0)
      continue;

    vector<SmartScan*> *pcc = itss->second.connectedComponents(0.03, 2000);
    vector<SmartScan*> cc = *pcc;
    for(unsigned int i=0; i<cc.size(); i++) {
      pair<int, SmartScan*> pr;
      pr.first = itss->first;
      pr.second = cc[i];
      ss_objs_.push_back(pr);
    }
  }

  objects_extracted_ = true;
}



int main(int argc, char **argv) {
  ros::init(argc, argv);
  usleep(500000);

  for(int i=1; i<argc; i++) {
    ros::node node("scene_labeler_stereo");
    usleep(500000);
    SceneLabelerStereo sls(&node);

    cout << "start" << endl;
    sls.loadMsgsFromFile(string(argv[i]));
    sls.processMsgs();
    cout << "finished" << endl;
    sls.publishAll();
    cout << "Press Enter to see next scene . . ."; cout.flush();
    cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    sls.node_->shutdown();
    node.shutdown();
  }

  usleep(500000);
  ros::fini();
}
