/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include <ros/ros.h>
#include <ros/console.h>

#include <gtest/gtest.h>

#include <tf/transform_listener.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/interactive_marker_client.h>

#define DBG_MSG( ... ) printf( __VA_ARGS__ ); printf("\n");
#define DBG_MSG_STREAM( ... )  std::cout << __VA_ARGS__ << std::endl;

using namespace interactive_markers;

int update_calls;
int init_calls;
int reset_calls;
int status_calls;


std::string reset_server_id;

typedef visualization_msgs::InteractiveMarkerInitConstPtr InitConstPtr;
typedef visualization_msgs::InteractiveMarkerUpdateConstPtr UpdateConstPtr;

InitConstPtr init_msg;
UpdateConstPtr update_msg;

void resetCount()
{
  update_calls = 0;
  init_calls = 0;
  reset_calls = 0;
  status_calls = 0;
  reset_server_id = "";
  init_msg.reset();
  update_msg.reset();
}

void updateCb( const UpdateConstPtr& msg )
{
  DBG_MSG("updateCb called");
  update_calls++;
  update_msg = msg;
}

void initCb( const InitConstPtr& msg )
{
  DBG_MSG("initCb called");
  init_calls++;
  init_msg = msg;
}

void statusCb( InteractiveMarkerClient::StatusT status,
    const std::string& server_id,
    const std::string& msg )
{
  DBG_MSG("statusCb called");
  status_calls++;
  DBG_MSG_STREAM( (int)status << " " << server_id << ": " << msg );
}

void resetCb( const std::string& server_id )
{
  DBG_MSG("resetCb called");
  reset_calls++;
  reset_server_id = server_id;
}

void waitMsg()
{
  for(int i=0;i<10;i++)
  {
    ros::spinOnce();
    usleep(1000);
  }
}

TEST(InteractiveMarkerClient, protocol)
{
  tf::TransformListener tf;

  // create an interactive marker server on the topic namespace simple_marker

  visualization_msgs::InteractiveMarker int_marker;
  int_marker.name = "marker1";
  int_marker.header.frame_id = "valid_frame";

  resetCount();

  interactive_markers::InteractiveMarkerClient client(tf, "valid_frame", "im_client_test");
  client.setInitCb( &initCb );
  client.setStatusCb( &statusCb );
  client.setResetCb( &resetCb );
  client.setUpdateCb( &updateCb );

  // Add one marker -> init should get called once
  DBG_MSG("----------------------------------------");

  InitConstPtr init_msg_out( new visualization_msgs::InteractiveMarkerInit() );
  init_msg_out->markers.push_back( int_marker );
  init_msg_out->seq_num=0;

  client.processInit( init_msg_out );

  ASSERT_EQ( update_calls, 0 );
  ASSERT_EQ( init_calls, 0 );
  ASSERT_EQ( reset_calls, 0 );

  client.update();

  ASSERT_EQ( update_calls, 0 );
  ASSERT_EQ( init_calls, 0 );
  ASSERT_EQ( reset_calls, 0 );

  UpdateConstPtr update_msg_out( new visualization_msgs::InteractiveMarkerUpdate() );
  update_msg_out->type = visualization_msgs::InteractiveMarkerUpdate::KEEP_ALIVE;
  client.processUpdate( update_msg_out );

  ASSERT_EQ( update_calls, 0 );
  ASSERT_EQ( init_calls, 0 );
  ASSERT_EQ( reset_calls, 0 );

  client.update();

  ASSERT_EQ( update_calls, 0 );
  ASSERT_EQ( init_calls, 1 );
  ASSERT_EQ( reset_calls, 0 );

  ASSERT_TRUE( init_msg );
  ASSERT_EQ( init_msg->markers.size(), 1 );
  ASSERT_EQ( init_msg->markers[0].name, "marker1" );
  ASSERT_EQ( init_msg->markers[0].header.frame_id, "valid_frame" );

  // Add another marker -> update should get called once
  DBG_MSG("----------------------------------------");

  resetCount();

  int_marker.name = "marker2";

  update_msg_out->type = visualization_msgs::InteractiveMarkerUpdate::UPDATE;
  update_msg_out->markers.push_back( int_marker );
  client.processUpdate( update_msg_out );

#if 0
  server.insert(int_marker);
  server.applyChanges();
  waitMsg();
  client.update();

  ASSERT_EQ( update_calls, 1 );
  ASSERT_EQ( init_calls, 0 );
  ASSERT_EQ( reset_calls, 0 );
  ASSERT_TRUE( update_msg );
  ASSERT_EQ( update_msg->markers.size(), 1 );
  ASSERT_EQ( update_msg->markers[0].name, "marker2" );

  // Make marker tf info invalid -> connection should be reset
  DBG_MSG("----------------------------------------");

  resetCount();

  int_marker.header.frame_id = "invalid_frame";

  server.insert(int_marker);
  server.applyChanges();
  waitMsg();
  client.update();

  ASSERT_EQ( update_calls, 0 );
  ASSERT_EQ( init_calls, 0 );
  ASSERT_EQ( reset_calls, 1 );
  ASSERT_EQ( reset_server_id, "/im_client_test/test_server" );

  // Make marker tf info valid again -> connection should be successfully initialized again
  DBG_MSG("----------------------------------------");

  usleep(2000000);
  client.update();

  resetCount();

  int_marker.header.frame_id = "valid_frame";

  server.insert(int_marker);
  server.applyChanges();
  waitMsg();
  client.update();

  ASSERT_EQ( update_calls, 0 );
  ASSERT_EQ( init_calls, 1 );
  ASSERT_EQ( reset_calls, 0 );
  ASSERT_EQ( reset_server_id, "/im_client_test/test_server" );
#endif
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  ros::init(argc, argv, "im_client_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
