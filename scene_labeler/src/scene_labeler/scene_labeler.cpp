#include <std_msgs/PointCloudFloat32.h>
#include <ros/node.h>
#include <std_msgs/Empty.h>
#include <logsnarf/logsnarf.h>

using namespace std;

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
};


void emptycallback(ros::Time t) {
}

int main(int argc, char **argv) {
  LogSnarfer s;
  LogPlayer<std_msgs::PointCloudFloat32> cloud;
  std_msgs::Empty shutter;

  int x=0;
  ros::init(x, NULL);

  if(argc!=2) {
    cerr << "usage: scene_labeler bagfile" << endl;
    return 1;
  }
  
  s.addLog(cloud, string(argv[1]), &emptycallback);
  s.snarf();

  //Cloud should now contain the data in the message.
  scene_labeler lab;

  //Why does this only work if we publish multiple times?
  for(int i=0; i<10; i++) {
    lab.publish("shutter", shutter);
  }
  usleep(1000000);
  lab.publish("full_cloud", cloud);
  cout << "Published cloud once." << endl;


  //Why does this only work if we publish multiple times?
  for(int i=0; i<10; i++) {
    lab.publish("full_cloud", cloud);
  }
  cout << "Published cloud lots." << endl;
  
  ros::fini();

  return 0;
}
