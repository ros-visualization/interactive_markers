#include <stdio.h>
#include <scene_labeler.h>

using namespace std;

int main(int argc, char **argv) {
  ros::init(argc, argv);
  usleep(500000);

  for(int i=1; i<argc; i++) {
    scene_labeler sl;
    sl.processMsgs(string(argv[i]));
    sl.publishAll();
    cout << "Press Enter to continue . . .\n";
    cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    sl.shutdown();

  }

  usleep(500000);
  ros::fini();
}


/*
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
    std_msgs::PointCloud cloud = sl->loadLabeledFile(segmentedString, &labels);

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
      std_msgs::PointCloud msg = (*ccs)[i]->getPointCloud();
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
    std_msgs::PointCloud cloud = sl->loadLabeledFile(string(argv[2]), NULL);
    
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

    std_msgs::PointCloud finalcloudmsg = foreground.getPointCloud();

    std_msgs::VisualizationMarker mark;
    std_msgs::Point32 pt;
    float support = .1;
    float pixelsPerMeter = 250;

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


*/







  

//   void subtract(char *fg, char *bg, int label) {
//     cout << "Subtracting " << bg << " from " << fg << " and applying label " << label << " to the new points." << endl;

//     string foregroundString = string(fg);
//     string fgDir, fgFile;
//     SplitFilename(foregroundString, &fgDir, &fgFile);
//     string fgNum = fgFile.substr(0, fgFile.find_first_of("-"));

//     //Load the data and put them into SmartScans.
//     SmartScan foreground, foreground2, background;
//     foreground.setPoints(cloud1.get_pts_size(), cloud1.pts);
//     foreground2.setPoints(cloud1.get_pts_size(), cloud1.pts);
//     background.setPoints(cloud2.get_pts_size(), cloud2.pts);
//     foreground.crop(0,0,0,BOXSIZE,BOXSIZE,BOXSIZE);
//     background.crop(0,0,0,BOXSIZE,BOXSIZE,BOXSIZE);
//     foreground2.crop(0,0,0,BOXSIZE,BOXSIZE,BOXSIZE);

//     //Subtract the background.
//     time_t start, end, dif;
//     time(&start);
//     foreground.subtractScan(&background, CCTHRESH);
//     time(&end); dif = difftime(end,start);
//     cout << "Subtraction took " << dif << " seconds." << endl;

//     //Condition the data.
//     foreground.removeGrazingPoints(10);
//     vector<SmartScan*> *ccs;
//     ccs = foreground.connectedComponents(CCTHRESH, MINPTS);
//     cout << ccs->size() << " connected components were large enough." << endl;

//     //Put just the large-enough connected components into a final scan.
//     SmartScan final;
//     for(unsigned int i=0; i<ccs->size(); i++) {
//       final.addScan((*ccs)[i]);
//     }

//     //Publish the final scan.
//     std_msgs::PointCloud finalcloudmsg = final.getPointCloud();
//     publish("full_cloud", finalcloudmsg); 

//     //Save the scan to a file.
//     string segmentedString  = fgDir + string("/") + fgNum + string("-sg-full_cloud.txt");
//     ofstream f(segmentedString.c_str()); 
//     if(!f.is_open()) {
//       cerr << "Could not open temporary file." << endl;
//     }

//     std_msgs::Point32 pt;
//     vector<std_msgs::Point32> *nearby;
//     int outputlabel;
//     cout << "Writing " << foreground2.size() << " points to " << segmentedString << endl;
//     for(int i=0; i<foreground2.size(); i++) {
//       pt = foreground2.getPoint(i);
//       nearby = final.getPointsWithinRadius(pt.x, pt.y, pt.z, .01);
//       if(nearby->size() > 0) {
// 	outputlabel = label;
//       }
//       else { 
// 	outputlabel = 0;
//       }
      
//       f << pt.x << " " << pt.y << " " << pt.z << " " << 0 << " " << outputlabel << endl; //0 is a placeholder for intensity.
//     }
//     f.close();
//     cout << "Done." << endl;
//   }

//   //Load the points in filename that are labeled with one of labels.  If labels==NULL, then accept all.
//   std_msgs::PointCloud loadLabeledFile(string filename, vector<int> *labels=NULL) {
//     FILE *fp = fopen(filename.c_str(), "r");
//     if(!fp) {
//       cerr << "Could not open segmented file." << endl;
//     }

//     float x, y, z;
//     int intensity, label;    
//     int nLines = 0;

//     //If we are accepting all labels
//     if(labels == NULL) {
//       labels = new vector<int>;
//       while(fscanf(fp, "%f %f %f %d %d\n", &x, &y, &z, &intensity, &label) != EOF) {	
// 	nLines++;
// 	if(find(labels->begin(), labels->end(), label) == labels->end()) //If label is not in labels
// 	  labels->push_back(label);
//       }
//     }
//     //If we are only selecting a subset
//     else {
//       while(fscanf(fp, "%f %f %f %d %d\n", &x, &y, &z, &intensity, &label) != EOF) {
// 	if(find(labels->begin(), labels->end(), label) != labels->end()) //If label is in labels
// 	  nLines++;
//       }
//     }
    
//     cout << "Found " << nLines << " points that are interesting." << endl;

//     //Set up pointcloud message.
//     std_msgs::PointCloud cloud;    
//     cloud.set_pts_size(nLines);
//     cloud.set_chan_size(2);
//     //cloud.set_chan_size(1); //???????????????
//     cloud.chan[0].name = "intensity";
//     cloud.chan[0].set_vals_size(nLines);
//     cloud.chan[1].name = "label";
//     cloud.chan[1].set_vals_size(nLines);

//     //Fill it with data from the text file.
//     cout << "Reading in data.." << endl;
//     rewind(fp);
//     int i = 0;
//     while(fscanf(fp, "%f %f %f %d %d", &x, &y, &z, &intensity, &label) == 5) { 
//       if(find(labels->begin(), labels->end(), label) != labels->end()) {
// 	cloud.pts[i].x = x;
// 	cloud.pts[i].y = y;
// 	cloud.pts[i].z = z;
// 	cloud.chan[0].vals[i] = intensity;
// 	cloud.chan[1].vals[i] = label;
// 	i++;
//       }
//     }
    
//     return cloud;
//   }




  
//   scene_labeler() : ros::node("scene_labeler") 
//   {
//     advertise<std_msgs::PointCloud>("full_cloud");
//     advertise<std_msgs::Empty>("shutter");
//     advertise<std_msgs::VisualizationMarker>("visualizationMarker");
//   }

//   // -- For subtracting two point clouds.
//   scene_labeler(string file1, string file2) : ros::node("scene_labeler")
//   {
//     advertise<std_msgs::PointCloud>("full_cloud");
//     advertise<std_msgs::Empty>("shutter");
//     advertise<std_msgs::VisualizationMarker>("visualizationMarker");

//     lp1.open(file1, ros::Time(0));
//     lp1.addHandler<std_msgs::PointCloud>(string("full_cloud"), &copyData, (void*)(&cloud1), true);
//     lp1.nextMsg(); //cloud1 now contains the pointcloud.

//     lp2.open(file2, ros::Time(0));
//     lp2.addHandler<std_msgs::PointCloud>(string("full_cloud"), &copyData, (void*)(&cloud2), true);
//     lp2.nextMsg(); //cloud2 now contains the pointcloud.
//   }
