#include "ros/node.h"
#include "rostools/Log.h"

///ROS node
ros::node *m_rosNode;
///Message used for viewing ros error messages
rostools::Log rosErrMsg;

///Sends roserr messages for testing purposes
int main( int argc, char** argv )
{
	ros::init(argc,argv);
	m_rosNode = new ros::node("roserr_send_test");
	m_rosNode->advertise<rostools::Log>("/rosout",4);
	std::string input;
	while(true)
	{
		std::cin >> input;
		for(short i = 1; i < 40; i = i+10)
		{
			rosErrMsg.level = i;
			rosErrMsg.name = "roserrTester";
			rosErrMsg.msg = "I'm testing rosout.  Does wordwrap work?  I don't know, hopefully this long string will find out.\n";
			m_rosNode->publish("/rosout",rosErrMsg);
			for(double a = -9999;a < 9999; a = a+.01)
			{

			}
		}
		if(input == "quit")
			break;
	}
	ros::fini();
}
