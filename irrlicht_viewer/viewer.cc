#include "ExampleRenderClass.hh"

int main(int argc, char ** argv)
{
  ros::init(argc, argv);

  ros::node myNode("DemoRenderer",ros::node::ANONYMOUS_NAME);

  ExampleRender* myRenderer = new ExampleRender(myNode);

  while(myRenderer->isEnabled() && myNode.ok())
    {
      sleep(1);
    }

  ros::fini();

  return 0;
}
