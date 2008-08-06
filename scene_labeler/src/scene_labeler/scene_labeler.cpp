#include <iostream>
#include <vector>
#include <algorithm>
#include <std_msgs/PointCloudFloat32.h>
#include <ros/node.h>
#include <std_msgs/Empty.h>
#include <std_msgs/VisualizationMarker.h>
#include "logging/LogPlayer.h"

#include <smartScan.h>
#include <time.h>


using namespace std;

#define MINPTS 50
#define CCTHRESH .01 //In meters.
#define BOXSIZE 3 //In meters.

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
  if (m != 0) {
    *((std_msgs::PointCloudFloat32*)(n)) = *((std_msgs::PointCloudFloat32*)(m));
    cout << "Copying data." << endl;
  }
}

class scene_labeler : public ros::node
{
public:
  
  scene_labeler() : ros::node("scene_labeler") 
  {
    advertise<std_msgs::PointCloudFloat32>("full_cloud");
    advertise<std_msgs::Empty>("shutter");
    advertise<std_msgs::VisualizationMarker>("visualizationMarker");
  }

  scene_labeler(string file1, string file2) : ros::node("scene_labeler")
  {
    advertise<std_msgs::PointCloudFloat32>("full_cloud");
    advertise<std_msgs::Empty>("shutter");
    advertise<std_msgs::VisualizationMarker>("visualizationMarker");

    player1.open(file1, ros::Time(0));
    player1.addHandler<std_msgs::PointCloudFloat32>(string("full_cloud"), &copyData, (void*)(&cloud1), true);
    player1.nextMsg(); //cloud1 now contains the pointcloud.

    player2.open(file2, ros::Time(0));
    player2.addHandler<std_msgs::PointCloudFloat32>(string("full_cloud"), &copyData, (void*)(&cloud2), true);
    player2.nextMsg(); //cloud2 now contains the pointcloud.
  }

  LogPlayer player1, player2;
  std_msgs::PointCloudFloat32 cloud1, cloud2;

  void subtract(char *fg, char *bg, int label) {
    cout << "Subtracting " << bg << " from " << fg << " and applying label " << label << " to the new points." << endl;

    string foregroundString = string(fg);
    string fgDir, fgFile;
    SplitFilename(foregroundString, &fgDir, &fgFile);
    string fgNum = fgFile.substr(0, fgFile.find_first_of("-"));

    //Load the data and put them into SmartScans.
    SmartScan foreground, foreground2, background;
    foreground.setPoints(cloud1.get_pts_size(), cloud1.pts);
    foreground2.setPoints(cloud1.get_pts_size(), cloud1.pts);
    background.setPoints(cloud2.get_pts_size(), cloud2.pts);
    foreground.crop(0,0,0,BOXSIZE,BOXSIZE,BOXSIZE);
    background.crop(0,0,0,BOXSIZE,BOXSIZE,BOXSIZE);
    foreground2.crop(0,0,0,BOXSIZE,BOXSIZE,BOXSIZE);

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
    cout << ccs->size() << " connected components were large enough." << endl;

    //Put just the large-enough connected components into a final scan.
    SmartScan final;
    for(unsigned int i=0; i<ccs->size(); i++) {
      final.addScan((*ccs)[i]);
    }

    //Publish the final scan.
    std_msgs::PointCloudFloat32 finalcloudmsg = final.getPointCloud();
    publish("full_cloud", finalcloudmsg); 

    //Save the scan to a file.
    string segmentedString  = fgDir + string("/") + fgNum + string("-sg-full_cloud.txt");
    ofstream f(segmentedString.c_str()); 
    if(!f.is_open()) {
      cerr << "Could not open temporary file." << endl;
    }

    std_msgs::Point3DFloat32 pt;
    vector<std_msgs::Point3DFloat32> *nearby;
    int outputlabel;
    cout << "Writing " << foreground2.size() << " points to " << segmentedString << endl;
    for(int i=0; i<foreground2.size(); i++) {
      pt = foreground2.getPoint(i);
      nearby = final.getPointsWithinRadius(pt.x, pt.y, pt.z, .01);
      if(nearby->size() > 0) {
	outputlabel = label;
      }
      else { 
	outputlabel = 0;
      }
      
      f << pt.x << " " << pt.y << " " << pt.z << " " << 0 << " " << outputlabel << endl; //0 is a placeholder for intensity.
    }
    f.close();
    cout << "Done." << endl;
  }

  //Load the points in filename that are labeled with one of labels.  If labels==NULL, then accept all.
  std_msgs::PointCloudFloat32 loadLabeledFile(string filename, vector<int> *labels=NULL) {
    FILE *fp = fopen(filename.c_str(), "r");
    if(!fp) {
      cerr << "Could not open segmented file." << endl;
    }

    float x, y, z;
    int intensity, label;    
    int nLines = 0;

    //If we are accepting all labels
    if(labels == NULL) {
      labels = new vector<int>;
      while(fscanf(fp, "%f %f %f %d %d\n", &x, &y, &z, &intensity, &label) != EOF) {	
	nLines++;
	if(find(labels->begin(), labels->end(), label) == labels->end()) //If label is not in labels
	  labels->push_back(label);
      }
    }
    //If we are only selecting a subset
    else {
      while(fscanf(fp, "%f %f %f %d %d\n", &x, &y, &z, &intensity, &label) != EOF) {
	if(find(labels->begin(), labels->end(), label) != labels->end()) //If label is in labels
	  nLines++;
      }
    }
    
    cout << "Found " << nLines << " points that are interesting." << endl;

    //Set up pointcloud message.
    std_msgs::PointCloudFloat32 cloud;    
    cloud.set_pts_size(nLines);
    cloud.set_chan_size(2);
    //cloud.set_chan_size(1); //???????????????
    cloud.chan[0].name = "intensity";
    cloud.chan[0].set_vals_size(nLines);
    cloud.chan[1].name = "label";
    cloud.chan[1].set_vals_size(nLines);

    //Fill it with data from the text file.
    cout << "Reading in data.." << endl;
    rewind(fp);
    int i = 0;
    while(fscanf(fp, "%f %f %f %d %d", &x, &y, &z, &intensity, &label) == 5) { 
      if(find(labels->begin(), labels->end(), label) != labels->end()) {
	cloud.pts[i].x = x;
	cloud.pts[i].y = y;
	cloud.pts[i].z = z;
	cloud.chan[0].vals[i] = intensity;
	cloud.chan[1].vals[i] = label;
	i++;
      }
    }
    
    return cloud;
  }
};


int main(int argc, char **argv) {
  int x=0;
  ros::init(x, NULL);
  usleep(500000);

  scene_labeler *sl;

  // ------------------------------------------
  // No option: If we just get one bag, load the data and publish it.
  // ------------------------------------------
  if(argc==2) { 
    sl = new scene_labeler(string(argv[1]), string(argv[1]));
    sl->publish("full_cloud", sl->cloud1);
    cout << "Publishing " << sl->cloud1.get_pts_size() << " points." << endl;
  }

  // ------------------------------------------
  //-s option: subtract one from the other and publish the difference.
  //           Also, save an intermediate file which can be loaded with the -l option.
  // ------------------------------------------
  else if(argc==4 && !strcmp(argv[1], "-s")) {
    sl = new scene_labeler(string(argv[2]), string(argv[3]));
    int label = -1; //i.e., to be labeled by a human later.
    sl->subtract(argv[2], argv[3], label);
  }


  // ------------------------------------------
  //--label option: subtract foreground from background, then label the connected 
  //                 components in the segmented pointcloud with the same label.
  // ------------------------------------------
  else if(argc==5 && !strcmp(argv[1], "--label")) {
    sl = new scene_labeler(string(argv[3]), string(argv[4]));
    int label = atoi(argv[2]);
    sl->subtract(argv[3], argv[4], label);
  }

  // ------------------------------------------
  //--handlabel option: label the connected components in the segmented pointcloud.
  // ------------------------------------------
  else if(argc==3 && !strcmp(argv[1], "--handlabel")) {
    cout << "Labeling..." << endl;

    //Do filename parsing to get the segmented and labeled filenames.
    string segmentedString = string(argv[2]);
    string sgDir, sgFile;
    SplitFilename(segmentedString, &sgDir, &sgFile);
    string sgNum = sgFile.substr(0, sgFile.find_first_of("-"));
    string labeledString = sgDir + string("/") + sgNum + string("-labeled-full_cloud.txt");
    cout << "saving to " << labeledString << endl;
   
    //Load the data.
    sl = new scene_labeler();
    vector<int> labels(1);
    labels[0] = -1;  //Load only those points that still need to be labeled.
    std_msgs::PointCloudFloat32 cloud = sl->loadLabeledFile(segmentedString, &labels);

    //Publish the data to make sure it looks right.
    sl->publish("full_cloud", cloud);
    cout << "Press Enter to continue . . .\n";
    cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    
    //Put it into a smartScan.
    SmartScan foreground;
    foreground.setPoints(cloud.get_pts_size(), cloud.pts);

    //Get the segments.
    vector<SmartScan*> *ccs;
    ccs = foreground.connectedComponents(CCTHRESH, MINPTS);

    //Present each segment in turn and ask the user for a label.
    int lbl;
    for(unsigned int i=0; i<ccs->size(); i++) {
      std_msgs::PointCloudFloat32 msg = (*ccs)[i]->getPointCloud();
      sl->publish("full_cloud", msg);
      cout << "\n\n****  What is this?" << endl;
      system("cat labels.txt");
      cout << "Enter a number: ";
      cin >> lbl;
      cout << "\nLabeling as " << lbl << endl;
    }

    //TODO: Save file with appropriate labeling.
  }
  

  // ------------------------------------------
  //--display option: Show the labels on some data.
  // ------------------------------------------
  else if(argc==3 && !strcmp(argv[1], "--display")) {
    sl = new scene_labeler();
    std_msgs::PointCloudFloat32 cloud = sl->loadLabeledFile(string(argv[2]), NULL);
    
    //Get the unique labels.
    int label;
    vector<int> labels;
    for(unsigned int i=0; i<cloud.get_pts_size(); i++) {
      label = cloud.chan[1].vals[i];
      if(find(labels.begin(), labels.end(), label) == labels.end()) //If label is not in labels
	labels.push_back(label);
    }

    //For each label, highlight those points and publish.
    for(unsigned int i=0; i<labels.size(); i++) {
      if(labels[i] == 0) continue;
      for(unsigned int j=0; j<cloud.get_pts_size(); j++) {
	//	cout << "labels: " << cloud.chan[1].vals[j] << " " << labels[i] << endl; 
	if(cloud.chan[1].vals[j] == labels[i]) { 
	  //	  cout << "highlighting" << endl;
	  cloud.chan[0].vals[j] = 3900;
	}
	else { 
	  cloud.chan[0].vals[j] = 0;
	}
      }

      sl->publish("full_cloud", cloud);
      cout << "Highlighting label " << labels[i] << endl;
      cout << "Press Enter to continue . . .\n";
      cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
  }
  // ------------------------------------------
  //-si option: Demo spin images on a single scene.
  // ------------------------------------------
  else if(argc==3 && !strcmp(argv[1], "-si")) {
    cout << "Demoing spin image." << endl;
    sl = new scene_labeler(string(argv[2]), string(argv[2]));

    //Put message into a SmartScan and publish.
    SmartScan foreground;
    foreground.setPoints(sl->cloud1.get_pts_size(), sl->cloud1.pts);
    foreground.crop(0,0,0,1.2,1.2,1.2);

    std_msgs::PointCloudFloat32 finalcloudmsg = foreground.getPointCloud();

    std_msgs::VisualizationMarker mark;
    std_msgs::Point3DFloat32 pt;
    float support = .1;
    float pixelsPerMeter = 250;


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


    int randId;
    cout << " starting while loop" << endl;
    while(sl->ok()) {
      usleep(10000);
      sl->publish("full_cloud", finalcloudmsg);

      //Compute a random spin image.
      cout << "Computing spin image " << endl;
      scan_utils::Grid2D *psi = new scan_utils::Grid2D((int)(2*support*pixelsPerMeter), (int)(support*pixelsPerMeter));
      srand(time(NULL));
      randId = rand() % foreground.size();
      pt = foreground.getPoint(randId);
      //foreground.computeSpinImageFixedOrientation(*psi, pt.x, pt.y, pt.z, support, pixelsPerMeter);
      foreground.computeSpinImageNatural(*psi, pt.x, pt.y, pt.z, support, pixelsPerMeter);

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


      // **** DIRTY DEBUG
//         // -- Get the surface normal.
//         std_msgs::Point3DFloat32 normal = foreground.computePointNormal(pt.x, pt.y, pt.z, .02, 20);
// 	cout << "Point normal is " << normal.x << " " << normal.y << " " << normal.z << endl;

// 	// -- Get the nearby points and put them into their own SmartScan.
// 	std_msgs::PointCloudFloat32* cld = foreground.getPointsWithinRadiusPointCloud(pt.x, pt.y, pt.z, support*sqrt(2));
// 	SmartScan ss;
// 	ss.setFromRosCloud(*cld);
// 	cout << "support has " << ss.size() << " points." << endl;
	
// 	cout << "Press Enter to see support . . .\n";
// 	cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
// 	std_msgs::PointCloudFloat32 ssdisp = ss.getPointCloud();
// 	sl->publish("full_cloud", ssdisp);
	
// 	// -- Setup libTF.
// 	libTF::TFVector tfn0, tfn1, tfn2;
// 	libTF::TransformReference tr;
// 	tfn0.frame = 13;
// 	tfn0.time = 0; 
// 	tfn0.x = normal.x;
// 	tfn0.y = normal.y;
// 	tfn0.z = normal.z;

// 	// -- Find the transformation that makes the surface normal point up.
// 	double pitch = atan2(normal.x, normal.z);
// 	tr.setWithEulers((unsigned int)13, (unsigned int)14, 0.0, 0.0, 0.0, 0.0, -pitch, 0.0, (libTF::TransformReference::ULLtime)0);
// 	cout << "  pitch is " << pitch << " from x = " << normal.x << " and z = " << normal.z << endl;

// 	tfn1 = tr.transformVector(14, tfn0);
// 	cout << "New normal is  x: " << tfn1.x << " y: " << tfn1.y << " z: " << tfn1.z << endl;
// 	float roll = atan2(tfn1.y, tfn1.z);
// 	cout << "  roll is " << roll << " from y = " << tfn1.y << " and z = " << tfn1.z << endl;
// 	tr.setWithEulers((unsigned int)14, (unsigned int)15, 0.0, 0.0, 0.0, 0.0, 0.0, roll, (libTF::TransformReference::ULLtime)0);
// 	tfn2 = tr.transformVector(15, tfn1);
// 	cout << "Transformed normal is (should be 0,0,1)  x: " << tfn2.x << " y: " << tfn2.y << " z: " << tfn2.z << endl;

// 	// -- Transform the spin image center point.
// 	libTF::TFPoint center0, center2;
// 	center0.frame = 13;
// 	center0.time = 0;
// 	center0.x = pt.x;
// 	center0.y = pt.y;
// 	center0.z = pt.z;
// 	center2 = tr.transformPoint(15, center0);

// 	// -- Transform the rest of the points in one shot and get the spin image.
// 	//ss.applyTransform(tr.getMatrix(13, 15, 0));
// 	libTF::TFPoint ptf, ptf2;
// 	std_msgs::Point3DFloat32 p;
// 	std_msgs::PointCloudFloat32 cld2;
// 	cld2.set_pts_size(ss.size());
// 	cld2.set_chan_size(1);
// 	cld2.chan[0].name = "intensities";
// 	cld2.chan[0].set_vals_size(ss.size());
// 	for (int j=0; j<ss.size(); j++) {
// 	  p = ss.getPoint(j);
// 	  ptf.frame = 13;
// 	  ptf.time = 0;
// 	  ptf.x = p.x;
// 	  ptf.y = p.y;
// 	  ptf.z = p.z;
// 	  ptf2 = tr.transformPoint(15, ptf);
// 	  cld2.pts[j].x = ptf2.x;
// 	  cld2.pts[j].y = ptf2.y;
// 	  cld2.pts[j].z = ptf2.z;
// 	  cld2.chan[0].vals[j] = 0;
// 	}
// 	SmartScan ss2;
// 	ss2.setFromRosCloud(cld2);
	

// 	// -- Display the new ptcld and marker.
// 	cout << "Press Enter to see rotated ptcld . . .\n";
// 	cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
// 	ssdisp = ss.getPointCloud();
// 	sl->publish("full_cloud", cld2);

// 	mark.action = 1;
// 	mark.x = center2.x;
// 	mark.y = center2.y;
// 	mark.z = center2.z;
// 	sl->publish("visualizationMarker", mark);

// 	ss2.computeSpinImageFixedOrientation(*psi, center2.x, center2.y, center2.z, support, pixelsPerMeter);

      //*** DONE DEBUG

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
      f.close();
      system(cmd);
      remove(filename);

      //Clean up.
      cout << "Press Enter to see next spin image . . .\n";
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
            "scene_labeler -s foreground background \t#subtracts background from foreground, publishes, and saves in bagfile1's directory to (number)-segmented.txt\n"
            "scene_labeler -l segmented.txt \t#Asks user for labels of connected components in segmented, \n"
            "\t\t\t\t\t\t  then applies labels to points in foreground and saves (number)-labeled.txt\n"
      "scene_labeler --label x foreground background \t #subtracts foreground from background and saves a  (number)-labeled.txt\n" << endl;

  }

   
  sleep(2);
  ros::fini();
  return 0;
}
