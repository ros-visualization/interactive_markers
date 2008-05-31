#include "PPILRender.hh"

int main(int argc, char ** argv)
{
  ros::init(argc, argv);

  PPILRender* myRenderer = new PPILRender();

  while(1)
    {
      sleep(1);
    }

  ros::fini();

  return 0;
}
