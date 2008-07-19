#include <iostream>
#include <vector>
#include <std_msgs/PointCloudFloat32.h>
#include <ros/node.h>
#include <std_msgs/Empty.h>
#include <logsnarf/logsnarf.h>
#include <smartScan.h>
#include <time.h>

using namespace std;

#define MINPTS = 100;
#define CCTHRESH = .01;


void SplitFilename (const string& str, string *dir, string *file)
{
  size_t found;
  found=str.find_last_of("/\\");
  *file = str.substr(found+1);
  *dir = str.substr(0,found);
}

void emptycallback(ros::Time t) {
}

class scene_labeler : public ros::node
{
public:
  scene_labeler() : ros::node("scene_labeler")
  {
    advertise<std_msgs::PointCloudFloat32>("full_cloud");
    advertise<std_msgs::Empty>("shutter");

    cloud[0] = new LogPlayer<std_msgs::PointCloudFloat32>();
    cloud[1] = new LogPlayer<std_msgs::PointCloudFloat32>();
}

  void broadcast_pointcloud(char *filename) {
  }
  
  void read_bag(char *filename, unsigned int cloudId) {
    s.addLog(*cloud[cloudId], string(filename), &emptycallback);
    s.snarf();   //Cloud should now contain the data in the message.
  }

  LogPlayer<std_msgs::PointCloudFloat32> *cloud[2];
  LogSnarfer s;
  std_msgs::Empty shutter;
};



int main(int argc, char **argv) {
  int x=0;
  ros::init(x, NULL);
  usleep(500000);

  scene_labeler sl;

  //Filename parsing on foreground.  Not for argc==2 case.
  string foregroundString = string(argv[2]);
  string fgDir, fgFile;
  SplitFilename(foregroundString, &fgDir, &fgFile);
  string fgNum = fgFile.substr(0, fgFile.find_first_of("-"));


  //  If we just get one bag, load the data and publish it.
  if(argc==2) { 
    sl.read_bag(argv[1], 0);
    sl.publish("full_cloud", *sl.cloud[0]);

//     //Put it into scan_utils.
//     SmartScan *ss = new SmartScan();
//     ss->setPoints(sl.cloud[1]->get_pts_size(), sl.cloud[1]->pts);
//     //ss->crop(0,0,0,1,1,1);
//     //ss->removeGrazingPoints(1);
//     vector<std_msgs::Point3DFloat32> *ptsInRadius = ss->getPointsWithinRadius(0, 0, 0, 10);

  
//     //Load a message with the modified cloud and publish.
//     std_msgs::PointCloudFloat32 c2;
//     c2.set_pts_size(ptsInRadius->size());
//     c2.set_chan_size(1);
//     c2.chan[0].name = "intensities";
//     c2.chan[0].set_vals_size(ptsInRadius->size());
//     for(unsigned int i=0; i<ptsInRadius->size(); i++) {
//       c2.pts[i].x = (*ptsInRadius)[i].x;
//       c2.pts[i].y = (*ptsInRadius)[i].y;
//       c2.pts[i].z = (*ptsInRadius)[i].z;
//       c2.chan[0].vals[i] = 0;
//     }
//     sl.publish("full_cloud", c2);
//     cout << "Published cloud." << endl;
//     delete ss;
  }
  
  //If we get 2 bags, subtract one from the other and publish the difference.
  else if(argc==4 && !strcmp(argv[1], "-s")) {
    cout << "Segmenting..." << endl;

    //Load the data.
    sl.read_bag(argv[1], 0);
    sl.read_bag(argv[2], 1);
    
    //Put them both into SmartScans.
    SmartScan foreground, background;
    foreground.setPoints(sl.cloud[0]->get_pts_size(), sl.cloud[0]->pts);
    background.setPoints(sl.cloud[1]->get_pts_size(), sl.cloud[1]->pts);
    //foreground.crop(0,0,0,1.2,1.2,1.2);
    //background.crop(0,0,0,1.2,1.2,1.2);

    //Subtract the background.
    cout << "starting bg subtr" << endl;
    time_t start,end;
    time(&start);
    foreground.subtractScan(&background, CCTHRESH);
    time(&end); double dif = difftime(end,start);
    cout << "Subtraction took " << dif << " seconds." << endl;

    foreground.removeGrazingPoints(10);
    vector<SmartScan*> *ccs;
    ccs = foreground.connectedComponents(CCTHRESH, MINPTS);


    //Put just the large-enough connected components into a final scan.
    SmartScan final;
    for(unsigned int i=0; i<ccs->size(); i++) {
      final.addScan((*ccs)[i]);
    }

    //Publish the final scan and save it to a file.
    std_msgs::PointCloudFloat32 finalcloudmsg = final.getPointCloud();
    string toSave = fgDir + string("/") + fgNum + string("-sg-full_cloud.bag"); //TODO: make this general.
    dustbuster *db = new dustbuster(string("full_cloud"), fgDir, string("sg"), toSave); //TODO: add a new dustbuster constructor.
    sl.publish("full_cloud", finalcloudmsg); //This will be saved by dustbuster.
    delete db;
  }

  else if(argc==4 && !strcmp(argv[1], "-l")) {
    cout << "Labeling..." << endl;

    //Do filename parsing to get the segmented filename.
    string segmentedString = string(argv[3]);
    string sgDir, sgFile;
    SplitFilename(segmentedString, &sgDir, &sgFile);
    string sgNum = sgFile.substr(0, sgFile.find_first_of("-"));
    if(sgNum.compare(fgNum)) {
      cout << "Foreground and segmented file do not have the same number!" << endl;
    }
    string labeledFile = fgDir + string("/") + fgNum + string("-segmented.bag");
    cout << "saving to " << labeledFile << endl;
   
    //Load the data.
    sl.read_bag(argv[2], 0);
    sl.read_bag(argv[3], 1);
    
    //Put them both into SmartScans.
    SmartScan foreground, segmented;
    foreground.setPoints(sl.cloud[0]->get_pts_size(), sl.cloud[0]->pts);
    segmented.setPoints(sl.cloud[1]->get_pts_size(), sl.cloud[1]->pts);

    //Get the segments.
    vector<SmartScan*> *ccs;
    ccs = foreground.connectedComponents(CCTHRESH, MINPTS);

    //Present each segment in turn and ask the user for a label.
    for(unsigned int i=0; i<ccs->size(); i++) {
      std_msgs::PointCloudFloat32 msg = ccs[i].getPointCloud();
      sl.publish("full_cloud", msg);
      cout << "Pausing..." << endl;
      int dummy; cin >> dummy; //TODO: Actually ask for and apply a label.
    }
  }

  else {
    cout << "usage:\n"
            "scene_labeler bagfile \t\t\t#publishes messages in bagfile.\n"
            "scene_labeler -s foreground background \t#subtracts background from foreground, publishes, and saves in bagfile1's directory to (number)-segmented.bag\n"
            "scene_labeler -l foreground segmented \t#Asks user for labels of connected components in segmented, \n"
            "\t\t\t\t\t\t  then applies labels to points in foreground and saves (number)-labeled.bag\n" << endl;
  }

    
  cout << "Finished successfully!" << endl;

  usleep(500000);
  ros::fini();
  return 0;
}




/* Old debugging crap.
  const int SLEEPTIME = 100000;
  //This does not work.
//   scene_labeler lab;
//   lab.publish("shutter", lab.shutter);

  //This works with SLEEPTIME = 1000.
//   scene_labeler lab;
//   usleep(SLEEPTIME);
//   lab.publish("shutter", lab.shutter);

  //This works, presumably because read_bag takes time.
//   scene_labeler lab;
//   lab.read_bag(argv[1]);
//   lab.publish("full_cloud", lab.cloud);

  //This only sends 2.
//   scene_labeler lab;
//   usleep(SLEEPTIME);
//   for(int i=0; i<10; i++) {
//     lab.publish("shutter", lab.shutter);
//   }

  //This sends all 10 with SLEEPTIME=1000.
//   scene_labeler lab;
//   usleep(SLEEPTIME);
//   for(int i=0; i<10; i++) {
//     lab.publish("shutter", lab.shutter);
//     usleep(SLEEPTIME);
//   }
  
  //This sends about half with SLEEPTIME=1000.
  //Maybe 3/4 with SLEEPTIME=10000.
  //Works well with SLEEPTIME=100000.
//   scene_labeler lab;
//   usleep(SLEEPTIME);
//   lab.read_bag(argv[1]);
//   for(int i=0; i<10; i++) {
//     lab.publish("full_cloud", lab.cloud);
//     usleep(SLEEPTIME);
//   }
*/
