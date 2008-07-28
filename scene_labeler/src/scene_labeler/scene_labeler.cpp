#include <iostream>
#include <vector>
#include <std_msgs/PointCloudFloat32.h>
#include <ros/node.h>
#include <std_msgs/Empty.h>
#include <std_msgs/VisualizationMarker.h>
#include "logging/LogPlayer.h"

#include <smartScan.h>
#include <time.h>


using namespace std;

#define MINPTS 100
#define CCTHRESH .01


void SplitFilename (const string& str, string *dir, string *file)
{
  size_t found;
  found=str.find_last_of("/\\");
  *file = str.substr(found+1);
  *dir = str.substr(0,found);
}

void emptycallback(ros::Time t) {
}


void copyData(string name, ros::msg* m, ros::Time t, void* n)
{
  if (m != 0)
    *((std_msgs::PointCloudFloat32*)(n)) = *((std_msgs::PointCloudFloat32*)(m));
}

class scene_labeler : public ros::node
{
public:
  
  scene_labeler(string file1, string file2) : ros::node("scene_labeler")
  {
    advertise<std_msgs::PointCloudFloat32>("full_cloud");
    advertise<std_msgs::Empty>("shutter");
    advertise<std_msgs::VisualizationMarker>("visualizationMarker");

    player1.open(file1, ros::Time(0));
    player1.addHandler<std_msgs::PointCloudFloat32>(string("/full_cloud"), &copyData, (void*)(&cloud1), true);
    player1.nextMsg(); //cloud1 now contains the pointcloud.

    player2.open(file2, ros::Time(0));
    player2.addHandler<std_msgs::PointCloudFloat32>(string("/full_cloud"), &copyData, (void*)(&cloud2), true);
    player2.nextMsg(); //cloud2 now contains the pointcloud.
  }

  LogPlayer player1, player2;
  std_msgs::PointCloudFloat32 cloud1, cloud2;

};


int main(int argc, char **argv) {
  int x=0;
  ros::init(x, NULL);
  usleep(500000);

  scene_labeler *sl;

  //Filename parsing on foreground.  Not for argc==2 case.

  //  If we just get one bag, load the data and publish it.
  if(argc==2) { 
    sl = new scene_labeler(string(argv[1]), string(argv[1]));
    sl->publish("full_cloud", sl->cloud1);

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

  //-s option: subtract one from the other and publish the difference.
  else if(argc==4 && !strcmp(argv[1], "-s")) {
    cout << "Segmenting..." << endl;
    
    string foregroundString = string(argv[2]);
    string fgDir, fgFile;
    SplitFilename(foregroundString, &fgDir, &fgFile);
    string fgNum = fgFile.substr(0, fgFile.find_first_of("-"));


    //Load the data.
    sl = new scene_labeler(string(argv[2]), string(argv[3]));
    
    //Put them both into SmartScans.
    SmartScan foreground, foreground2, background, background2;
    foreground.setPoints(sl->cloud1.get_pts_size(), sl->cloud1.pts);
    foreground2.setPoints(sl->cloud1.get_pts_size(), sl->cloud1.pts);
    background.setPoints(sl->cloud2.get_pts_size(), sl->cloud2.pts);
    background2.setPoints(sl->cloud2.get_pts_size(), sl->cloud2.pts);
    foreground.crop(0,0,0,1.2,1.2,1.2);
    foreground2.crop(0,0,0,1.2,1.2,1.2);
    background.crop(0,0,0,1.2,1.2,1.2);
    background2.crop(0,0,0,1.2,1.2,1.2);

    //Subtract the background.
    time_t start, end, dif;
    time(&start);
    foreground.subtractScan(&background, CCTHRESH);
    time(&end); dif = difftime(end,start);
    cout << "Subtraction took " << dif << " seconds." << endl;

    //Condition the data.
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
//     string toSave = fgDir + string("/") + fgNum + string("-sg-full_cloud.bag"); //TODO: make this general.
//     string topic = string("full_cloud");
//     dustbuster *db = new dustbuster(topic, toSave);
    sl->publish("full_cloud", finalcloudmsg); //This will be saved by dustbuster.
//     delete db;
  }

  /*
  //*****
  //-l option: label the connected components in the segmented pointcloud.
  else if(argc==4 && !strcmp(argv[1], "-l")) {
    cout << "Labeling..." << endl;

    string foregroundString = string(argv[2]);
    string fgDir, fgFile;
    SplitFilename(foregroundString, &fgDir, &fgFile);
    string fgNum = fgFile.substr(0, fgFile.find_first_of("-"));

    //Do filename parsing to get the segmented filename.
    string segmentedString = string(argv[2]);
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
      std_msgs::PointCloudFloat32 msg = (*ccs)[i]->getPointCloud();
      sl.publish("full_cloud", msg);
      cout << "Pausing..." << endl;
      int dummy; cin >> dummy; //TODO: Actually ask for and apply a label.
    }
  }
  */

  //-si option: Demo spin images on a single scene.
  else if(argc==3 && !strcmp(argv[1], "-si")) {
    cout << "Demoing spin image." << endl;
    sl = new scene_labeler(string(argv[2]), string(argv[2]));

    //Put message into a SmartScan and publish.
    SmartScan foreground;
    foreground.setPoints(sl->cloud1.get_pts_size(), sl->cloud1.pts);
    foreground.crop(0,0,0,1.2,1.2,1.2);

    std_msgs::PointCloudFloat32 finalcloudmsg = foreground.getPointCloud();
    sl->publish("full_cloud", finalcloudmsg);

    std_msgs::VisualizationMarker mark;
    std_msgs::Point3DFloat32 pt;
    float support = .1;
    float pixelsPerMeter = 250;

    int randId;
    cout << " starting while loop" << endl;
    while(sl->ok()) {
      usleep(10000);

      //Do speed testing.
//       int iters = 1000;
//       time_t start,end;
//       time(&start);
//       scan_utils::Grid2D *psitest = new scan_utils::Grid2D((int)(2*support*pixelsPerMeter), (int)(support*pixelsPerMeter));
//       for(int iter=0; iter<iters; iter++) {
// 	srand(time(NULL));
// 	randId = rand() % foreground.size();
// 	pt = foreground.getPoint(randId);
// 	foreground.computeSpinImageFixedOrientation(*psitest, pt.x, pt.y, pt.z, support, pixelsPerMeter);
//       }
//       time(&end); double dif = difftime(end,start);
//       cout << "Computing " << iters << " spin images took " << dif << " seconds." << endl;
//       delete psitest;

      //Compute a random spin image.
      cout << "Computing spin image " << endl;
      scan_utils::Grid2D *psi = new scan_utils::Grid2D((int)(2*support*pixelsPerMeter), (int)(support*pixelsPerMeter));
      srand(time(NULL));
      randId = rand() % foreground.size();
      pt = foreground.getPoint(randId);
      foreground.computeSpinImageFixedOrientation(*psi, pt.x, pt.y, pt.z, support, pixelsPerMeter);


      cout << "markering" << endl;
      //Put special symbol in pr2_gui.
      cout << "Putting a marker at " << pt.x << " " << pt.y << " " << pt.z << endl;
      mark.id = 1000 + randId;
      mark.type = 1;
      mark.action = 0;
      mark.x = pt.x;
      mark.y = pt.y;
      mark.z = pt.z;
      mark.roll = 0;
      mark.pitch = 0;
      mark.yaw = 0;
      mark.xScale = .02;
      mark.yScale = .02;
      mark.zScale = .02;
      mark.alpha = 255;
      mark.r = 0;
      mark.g = 255;
      mark.b = 0;
      mark.text = string("");
      sl->publish("visualizationMarker", mark);

      //Display the spin image with gnuplot.
      char cmd[] = "echo \'set pm3d map; set size ratio -1; splot \"gnuplot_tmp\"\' | gnuplot -persist";
      char filename[] = "gnuplot_tmp";
      ofstream f(filename); 
      if(!f.is_open()) {
	cerr << "Could not open temporary file." << endl;
      }

      int height, width;
      psi->getSize(height, width);
      for(int h=height-1; h>-1; h--) {
	for(int w=0; w<width; w++) {
	  f << psi->getElement(h, w) << endl;
	}
	f << endl;
      }
      system("killall gnuplot");
      int status = system(cmd);
      remove(filename);

      //Clean up.
      cout << "Press Enter to continue . . .\n";
      cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      mark.action = 2;
      usleep(100000);
      sl->publish("visualizationMarker", mark);
      usleep(100000);
    }
  }
  else {
    cout << "usage:\n"
            "scene_labeler bagfile \t\t\t#publishes messages in bagfile.\n"
            "scene_labeler -s foreground background \t#subtracts background from foreground, publishes, and saves in bagfile1's directory to (number)-segmented.bag\n"
            "scene_labeler -l foreground segmented \t#Asks user for labels of connected components in segmented, \n"
            "\t\t\t\t\t\t  then applies labels to points in foreground and saves (number)-labeled.bag\n" << endl;
  }

   
  usleep(500000);
  ros::fini();
  return 0;
}
