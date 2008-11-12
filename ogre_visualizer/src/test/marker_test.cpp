#include "ros/node.h"

#include "std_msgs/VisualizationMarker.h"

int main( int argc, char** argv )
{
  ros::init( argc, argv );

  ros::node* node = new ros::node( "MarkerTest", ros::node::DONT_HANDLE_SIGINT );

  while ( !node->ok() )
  {
    usleep( 10000 );
  }

  node->advertise<std_msgs::VisualizationMarker>( "visualizationMarker", 0 );

  usleep( 1000000 );

  for ( int i = -50; i < 50; ++i )
  {
    std_msgs::VisualizationMarker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time((uint64_t)0ULL);
    marker.id = i;
    marker.type = 1;
    marker.action = 0;
    marker.x = 1;
    marker.y = (i*2);
    marker.z = 0;
    marker.yaw = 0.0;
    marker.pitch = 0.0;
    marker.roll = 0.0;
    marker.xScale = 1;
    marker.yScale = 1;
    marker.zScale = 1;
    marker.alpha = 100;
    marker.r = 0;
    marker.g = 255;
    marker.b = 0;
    node->publish( "visualizationMarker", marker );
  }

  usleep( 1000000 );

  node->shutdown();
  delete node;

  ros::fini();
}
