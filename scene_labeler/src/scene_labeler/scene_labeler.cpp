#include <std_msgs/PointCloudFloat32.h>
#include <ros/node.h>
#include <std_msgs/Empty.h>
#include <logsnarf/logsnarf.h>

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
  const int SLEEPTIME = 50000;

  int x=0;
  ros::init(x, NULL);

  if(argc!=2) {
    cerr << "usage: scene_labeler bagfile" << endl;
    return 1;
  }


  //*****   The following snippets were tested with SLEEPTIME = 500000 */
  //This DOES NOT work without the sleep after publishing.
//   scene_labeler lab;
//   lab.read_bag(argv[1]);
//   usleep(SLEEPTIME);
//   lab.publish("full_cloud", lab.cloud);

  //This does work.
  //SLEEPTIME=50000 also works, but SLEEPTIME=5000 does not.
  scene_labeler lab;
  lab.read_bag(argv[1]);
  usleep(SLEEPTIME);
  lab.publish("full_cloud", lab.cloud);
  usleep(SLEEPTIME);

  //This works.
//   scene_labeler lab;
//   usleep(SLEEPTIME);
//   lab.publish("shutter", lab.shutter);

  //This only sends 2.
//   scene_labeler lab;
//   usleep(SLEEPTIME);
//   for(int i=0; i<10; i++) {
//     lab.publish("shutter", lab.shutter);
//   }

  //This sends all 10.
  //At SLEEPTIME=500, this gets flaky.  SLEEPTIME=5000 drops a few occasionally, and SLEEPTIME=50000 works well.
//   scene_labeler lab;
//   usleep(SLEEPTIME);
//   for(int i=0; i<10; i++) {
//     lab.publish("shutter", lab.shutter);
//     usleep(SLEEPTIME);
//   }
  
  //This sends all 10.
  //SLEEPTIME=50000 drops a few occasionally.
//   scene_labeler lab;
//   usleep(SLEEPTIME);
//   lab.read_bag(argv[1]);
//   for(int i=0; i<10; i++) {
//     lab.publish("full_cloud", lab.cloud);
//     usleep(SLEEPTIME);
//   }
  

//This works SOMETIMES.
//   scene_labeler lab;
//   lab.publish("shutter", lab.shutter);

  ros::fini();
  return 0;
}
