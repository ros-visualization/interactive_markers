#include <std_msgs/PointCloudFloat32.h>
#include <ros/node.h>
#include <std_msgs/Empty.h>
#include <logsnarf/logsnarf.h>
#include <smartScan.h>

using namespace std;


void emptycallback(ros::Time t) {
}

class scene_labeler : public ros::node
{
public:
  scene_labeler() : ros::node("scene_labeler")
  {
    advertise<std_msgs::PointCloudFloat32>("full_cloud");
    advertise<std_msgs::Empty>("shutter");
  }

  void broadcast_pointcloud(char *filename) {
  }
  
  void read_bag(char *filename) {
    s.addLog(cloud, string(filename), &emptycallback);
    s.snarf();   //Cloud should now contain the data in the message.
  }


  

  LogPlayer<std_msgs::PointCloudFloat32> cloud;
  LogSnarfer s;
  std_msgs::Empty shutter;
};



int main(int argc, char **argv) {
  int x=0;
  ros::init(x, NULL);
  usleep(500000);

  if(argc!=2) {
    cerr << "usage: scene_labeler bagfile" << endl;
    return 1;
  }

  //  Load the data.
  scene_labeler sl;
  sl.read_bag(argv[1]);

  //Put it into scan_utils.
  SmartScan *ss = new SmartScan();
  ss->setPoints(sl.cloud.get_pts_size(), sl.cloud.pts);
  ss->crop(0,0,0,2,2,2);
  ss->removeGrazingPoints(1);
  vector<std_msgs::Point3DFloat32> *ptsInRadius = ss->getPointsWithinRadius(0, 0, 0, 10);

  
  //Load a message with the modified cloud and publish.
  std_msgs::PointCloudFloat32 c2;
  c2.set_pts_size(ptsInRadius->size());
  c2.set_chan_size(1);
  c2.chan[0].name = "intensities";
  c2.chan[0].set_vals_size(ptsInRadius->size());
  for(unsigned int i=0; i<ptsInRadius->size(); i++) {
    c2.pts[i].x = (*ptsInRadius)[i].x;
    c2.pts[i].y = (*ptsInRadius)[i].y;
    c2.pts[i].z = (*ptsInRadius)[i].z;
    c2.chan[0].vals[i] = 0;
  }
  sl.publish("full_cloud", c2);
  cout << "Published cloud." << endl;

  

  delete ss;
  cout << "deleting" << endl;

  //usleep(5000000);
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
