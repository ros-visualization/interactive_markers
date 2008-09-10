#include "ros/node.h"
#include "rostools/Log.h"

#include <time.h>
#include <sstream>

int main( int argc, char** argv )
{
  const char* topic = "/rosout";

	ros::init(argc,argv);
	ros::node* node = new ros::node("rosout_send_test");
	node->advertise<rostools::Log>(topic, 0);

	sleep( 1 );

	while( 1 )
	{
    for ( int i = 0; i < 100; ++i )
    {
      rostools::Log message;
      std::stringstream name;
      name << "rosout_send_test" << i;
      message.name = name.str();

      message.msg = "Testing Info";
      message.level = rostools::Log::INFO;
      node->publish( topic, message );

      message.msg = "Testing Warn";
      message.level = rostools::Log::WARN;
      node->publish( topic, message );

      message.msg = "Testing Debug";
      message.level = rostools::Log::DEBUG;
      node->publish( topic, message );

      message.msg = "Testing Error";
      message.level = rostools::Log::ERROR;
      node->publish( topic, message );

      message.msg = "Testing Fatal";
      message.level = rostools::Log::FATAL;
      node->publish( topic, message );

      usleep( 10000 );
    }
	}

	ros::fini();
}
