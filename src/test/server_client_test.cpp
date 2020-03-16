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

#include <chrono>
#include <thread>

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

void resetReceivedMsgs()
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
  DBG_MSG("resetCb( %s ) called", server_id.c_str() );
  reset_calls++;
  reset_server_id = server_id;
}

void waitMsg()
{
  for(int i=0;i<10;i++)
  {
    ros::spinOnce();
    std::this_thread::sleep_for(std::chrono::microseconds(1000));
  }
}

TEST(InteractiveMarkerServerAndClient, connect_tf_error)
{
  tf::TransformListener tf;

  // create an interactive marker server on the topic namespace simple_marker
  interactive_markers::InteractiveMarkerServer server("im_server_client_test","test_server",false);
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.name = "marker1";
  int_marker.header.frame_id = "valid_frame";

  waitMsg();

  resetReceivedMsgs();

  interactive_markers::InteractiveMarkerClient client(tf, "valid_frame", "im_server_client_test");
  client.setInitCb( &initCb );
  client.setStatusCb( &statusCb );
  client.setResetCb( &resetCb );
  client.setUpdateCb( &updateCb );

  // Add one marker -> init should get called once
  DBG_MSG("----------------------------------------");

  server.insert(int_marker);
  server.applyChanges();
  waitMsg();
  client.update();

  ASSERT_EQ( 0, update_calls  );
  ASSERT_EQ( 1, init_calls  );
  ASSERT_EQ( 0, reset_calls  );
  ASSERT_TRUE( init_msg );
  ASSERT_EQ( 1, init_msg->markers.size()  );
  ASSERT_EQ( "marker1", init_msg->markers[0].name  );
  ASSERT_EQ( "valid_frame", init_msg->markers[0].header.frame_id  );

  // Add another marker -> update should get called once
  DBG_MSG("----------------------------------------");

  resetReceivedMsgs();

  int_marker.name = "marker2";

  server.insert(int_marker);
  server.applyChanges();
  waitMsg();
  client.update();

  ASSERT_EQ( 1, update_calls  );
  ASSERT_EQ( 0, init_calls  );
  ASSERT_EQ( 0, reset_calls  );
  ASSERT_TRUE( update_msg );
  ASSERT_EQ( 1, update_msg->markers.size()  );
  ASSERT_EQ( 0, update_msg->poses.size()  );
  ASSERT_EQ( 0, update_msg->erases.size()  );
  ASSERT_EQ( "marker2", update_msg->markers[0].name  );

  // Make marker tf info invalid -> connection should be reset
  DBG_MSG("----------------------------------------");

  resetReceivedMsgs();

  int_marker.header.frame_id = "invalid_frame";

  server.insert(int_marker);
  server.applyChanges();
  waitMsg();
  client.update();

  ASSERT_EQ( 0, update_calls  );
  ASSERT_EQ( 0, init_calls  );
  ASSERT_EQ( 1, reset_calls  );
  ASSERT_TRUE( reset_server_id.find("/test_server") != std::string::npos );

  // Make marker tf info valid again -> connection should be successfully initialized again
  DBG_MSG("----------------------------------------");

  std::this_thread::sleep_for(std::chrono::microseconds(2000000));
  waitMsg();
  client.update();

  resetReceivedMsgs();

  int_marker.header.frame_id = "valid_frame";

  server.insert(int_marker);
  server.applyChanges();
  waitMsg();
  client.update();

  ASSERT_EQ( 0, update_calls  );
  ASSERT_EQ( 1, init_calls  );
  ASSERT_EQ( 0, reset_calls  );

  // Erase marker
  DBG_MSG("----------------------------------------");

  resetReceivedMsgs();

  server.erase("marker1");
  server.applyChanges();
  waitMsg();
  client.update();

  ASSERT_EQ( 1, update_calls  );
  ASSERT_EQ( 0, init_calls  );
  ASSERT_EQ( 0, reset_calls  );
  ASSERT_TRUE( update_msg );
  ASSERT_EQ( 0, update_msg->markers.size()  );
  ASSERT_EQ( 0, update_msg->poses.size()  );
  ASSERT_EQ( 1, update_msg->erases.size()  );
  ASSERT_EQ( "marker1", update_msg->erases[0]  );

  // Update pose
  DBG_MSG("----------------------------------------");

  resetReceivedMsgs();

  server.setPose( "marker2", int_marker.pose, int_marker.header );
  server.applyChanges();
  waitMsg();
  client.update();

  ASSERT_EQ( 1, update_calls  );
  ASSERT_EQ( 0, init_calls  );
  ASSERT_EQ( 0, reset_calls  );
  ASSERT_TRUE( update_msg );
  ASSERT_EQ( 0, update_msg->markers.size()  );
  ASSERT_EQ( 1, update_msg->poses.size()  );
  ASSERT_EQ( 0, update_msg->erases.size()  );
  ASSERT_EQ( "marker2", update_msg->poses[0].name  );
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  ros::init(argc, argv, "im_server_client_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
