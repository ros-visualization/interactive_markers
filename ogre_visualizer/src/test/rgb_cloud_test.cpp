#include "ros/node.h"

#include "std_msgs/PointCloud.h"

int main( int argc, char** argv )
{
  ros::init( argc, argv );

  ros::node* node = new ros::node( "RGBCloudTest", ros::node::DONT_HANDLE_SIGINT );

  while ( !node->ok() )
  {
    usleep( 10000 );
  }

  node->advertise<std_msgs::PointCloud>( "rgb_cloud_test", 0 );
  node->advertise<std_msgs::PointCloud>( "rgb_cloud_test2", 0 );

  usleep( 1000000 );

  while (1)
  {
    {
      std_msgs::PointCloud cloud;
      cloud.pts.resize(5);
      cloud.chan.resize(1);
      for ( int i = 0; i < 5; ++i )
      {
        cloud.pts[i].x = (float)i;
        cloud.pts[i].y = 0.0f;
        cloud.pts[i].z = 0.0f;
      }

      cloud.chan[0].name = "rgb";
      cloud.chan[0].vals.resize(5);

      int rgb = (0xff << 16);
      cloud.chan[0].vals[0] = *reinterpret_cast<float*>(&rgb);
      rgb = (0xff << 8);
      cloud.chan[0].vals[1] = *reinterpret_cast<float*>(&rgb);
      rgb = 0xff;
      cloud.chan[0].vals[2] = *reinterpret_cast<float*>(&rgb);
      rgb = (0xff << 16) | (0xff << 8);
      cloud.chan[0].vals[3] = *reinterpret_cast<float*>(&rgb);
      rgb = (0xff << 8) | 0xff;
      cloud.chan[0].vals[4] = *reinterpret_cast<float*>(&rgb);

      node->publish( "rgb_cloud_test", cloud );
    }

    {
      std_msgs::PointCloud cloud;
      cloud.pts.resize(5);
      cloud.chan.resize(3);
      for ( int i = 0; i < 5; ++i )
      {
        cloud.pts[i].x = (float)i;
        cloud.pts[i].y = 1.0f;
        cloud.pts[i].z = 0.0f;
      }

      cloud.chan[0].name = "r";
      cloud.chan[0].vals.resize(5);
      cloud.chan[1].name = "g";
      cloud.chan[1].vals.resize(5);
      cloud.chan[2].name = "b";
      cloud.chan[2].vals.resize(5);

      cloud.chan[0].vals[0] = 1.0f;
      cloud.chan[1].vals[0] = 0.0f;
      cloud.chan[2].vals[0] = 0.0f;

      cloud.chan[0].vals[1] = 0.0f;
      cloud.chan[1].vals[1] = 1.0f;
      cloud.chan[2].vals[1] = 0.0f;

      cloud.chan[0].vals[2] = 0.0f;
      cloud.chan[1].vals[2] = 0.0f;
      cloud.chan[2].vals[2] = 1.0f;

      cloud.chan[0].vals[3] = 1.0f;
      cloud.chan[1].vals[3] = 1.0f;
      cloud.chan[2].vals[3] = 0.0f;

      cloud.chan[0].vals[4] = 0.0f;
      cloud.chan[1].vals[4] = 1.0f;
      cloud.chan[2].vals[4] = 1.0f;

      node->publish( "rgb_cloud_test2", cloud );
    }

    usleep( 1000000 );
  }

  node->shutdown();
  delete node;

  ros::fini();
}
