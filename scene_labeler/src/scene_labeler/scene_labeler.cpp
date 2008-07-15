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
  const int SLEEPTIME = 100000;

  int x=0;
  ros::init(x, NULL);
  usleep(500000);

  if(argc!=2) {
    cerr << "usage: scene_labeler bagfile" << endl;
    return 1;
  }


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
  

  usleep(500000);
  ros::fini();
  return 0;
}
