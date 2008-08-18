#include "ros/node.h"
#include "rostools/Log.h"

///ROS node
ros::node *m_rosNode;
///Message used for viewing ros error messages
rostools::Log rosErrMsg;

int main( int argc, char** argv )
{
	ros::init(argc,argv);
	m_rosNode = new ros::node("roserr_send_test");
	m_rosNode->advertise<rostools::Log>("/roserr");
	std::string input;
	while(true)
	{
		std::cin >> input;
		for(short i = 1; i < 40; i = i+10)
		{
			rosErrMsg.level = i;
			rosErrMsg.name = "roserrTester";
			rosErrMsg.msg = "I'm testing roserr.  Does wordwrap work?  I don't know, hopefully this long string will find out.\n";
			m_rosNode->publish("/roserr",rosErrMsg);
			/*for(double a = -9999;a < 9999; a = a+.0001)
			{

			}*/
		}
		if(input == "quit")
			break;
	}
	ros::fini();
}
