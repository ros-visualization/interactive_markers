#include "ros/node.h"
#include "rosconsole/rosconsole.h"

#include <time.h>
#include <sstream>

int main( int argc, char** argv )
{
	ros::init(argc,argv);
	ros::node* node = new ros::node("rosout_send_test", ros::node::DONT_HANDLE_SIGINT);

	int count = 0;
	while( 1 )
	{
    for ( int i = 0; i < 1; ++i )
    {
      ROS_DEBUG("Debug%d", count);
      ROS_INFO("Info%d", count);
      ROS_WARN("Warn%d", count);
      ROS_ERROR("Error%d", count);
      ROS_FATAL("Fatal%d", count);
      ROS_INFO("Newline\nTest\nblah\nblah\nblah\nblah\nblah\n");

      usleep( 1000000 );

      ++count;
    }
	}

	ros::fini();
	delete node;
}
