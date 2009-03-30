#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <ros/node.h>
#include <ros/time.h>
#include "Finger.h"

#include <iostream>

#define NODE_NAME "TouchPublisher"
#define MSG_NAME "TouchEvent"
#define PORT 10374

// data received from TouchKit app on Windows machine
typedef struct FingerEvent
{
  int id;
  int eventType;
  int x, y;
}
FingerEvent;

// receives touch events from Windows machine
class TouchServer
{
    public:

        TouchServer( int port );
        ~TouchServer();

        bool isConnected();
        int getTouchEvent( FingerEvent &f );

    private:

        int sock;
        bool connected;

};

TouchServer::TouchServer( int port )
{
    struct sockaddr_in destination;

    // create socket
	destination.sin_family = AF_INET;
	sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (sock < 0)
	{
		std::cout << "Socket Creation FAILED!";
        connected = false;
		return;
	}

    // bind socket
    destination.sin_port = htons (PORT);
	destination.sin_addr.s_addr = INADDR_ANY;
	if (bind(sock, (struct sockaddr *)&destination, sizeof(destination))<0)
	{
		std::cout << "Binding Socket FAILED!";
        connected = false;
		return;
	}

    // test socket
    std::cout << "Listening on " << PORT << "...";
    std::cout.flush();
	if (listen(sock, 5)<0)
	{
		std::cout << "Listening on Socket FAILED!";
		connected = false;
		return;
	}

    // connect to TouchKit client on Windows machine
    struct sockaddr_in clientAddress;
	int clientSize = sizeof(clientAddress);
	int accepted = accept(sock, (struct sockaddr *)&clientAddress, (socklen_t *) &clientSize);
	if (accepted < 0)
	{
		std::cout << "Socket Connection FAILED!";
		connected = false;
		return;
	}
	sock = accepted;
	std::cout<< "Connection Established!\n";
    connected = true;
}

TouchServer::~TouchServer()
{
     if (sock > 0)
        close(sock);
}

bool TouchServer::isConnected()
{
    return connected;
}

int TouchServer::getTouchEvent( FingerEvent &f )
{
    return recv(sock, &f, sizeof(FingerEvent), 0);
}

// publishes touch event to ros
class TouchPublisher
{
    public:

        TouchPublisher( const char *name );
        void publishTouchEvent( const FingerEvent &f );
        bool isOk();

    private:

        ros::Node node;

};

TouchPublisher::TouchPublisher( const char *name ):
    node(name)
{
  node.advertise<visualization::Finger>(MSG_NAME, 100);
}

bool TouchPublisher::isOk()
{
    return node.ok();
}

void TouchPublisher::publishTouchEvent( const FingerEvent &f )
{
    visualization::Finger msg;
    msg.id = f.id;
    msg.x = f.x;
    msg.y = f.y;
    msg.eventType = f.eventType;
    node.publish( MSG_NAME , msg );
}

int main (int argc, char **argv)
{
    ros::init(argc, argv);

    TouchServer server( PORT );
    if (!server.isConnected())
        return -1;

    TouchPublisher publisher( NODE_NAME );
    FingerEvent f;
    while ( publisher.isOk() )
    {
      if (server.getTouchEvent(f) == -1)
	break;
      std::cout << f.x << ' ' << f.y << std::endl;
      publisher.publishTouchEvent( f );
    }

    return 0;
}

