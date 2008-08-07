#include "ros/node.h"

#include "std_msgs/VisualizationMarker.h"

int main( int argc, char** argv )
{
  ros::init( argc, argv );

  ros::node* node = new ros::node( "MarkerTest" );

  while ( !node->ok() )
  {
    usleep( 10000 );
  }

  node->advertise<std_msgs::VisualizationMarker>( "visualizationMarker" );

  std_msgs::VisualizationMarker marker;
  marker.id = 0;
  marker.type = 0;
  marker.action = 0;
  marker.x = 1;
  marker.y = 0;
  marker.z = 0;
  marker.yaw = 3.14 / 4.0;
  marker.pitch = 3.14 / 4.0;
  marker.roll = 0.0;
  marker.xScale = 1;
  marker.yScale = 0.1;
  marker.zScale = 0.1;
  marker.alpha = 255;
  marker.r = 0;
  marker.g = 255;
  marker.b = 0;
  node->publish( "visualizationMarker", marker );

  node->shutdown();
  delete node;

  ros::fini();
}
