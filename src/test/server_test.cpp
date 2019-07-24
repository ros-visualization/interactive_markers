// Copyright (c) 2011, Willow Garage, Inc.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of Willow Garage, Inc. nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <ros/ros.h>

#include <gtest/gtest.h>

#include <interactive_markers/interactive_marker_server.h>

TEST(InteractiveMarkerServer, addRemove)
{
  // create an interactive marker server on the topic namespace simple_marker
  interactive_markers::InteractiveMarkerServer server("im_server_test");

  // create an interactive marker for our server
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.name = "marker1";

  // create a valid pose
  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;

  // insert, apply, erase, apply
  server.insert(int_marker);
  ASSERT_TRUE(server.get("marker1", int_marker) );

  server.applyChanges();
  ASSERT_TRUE(server.get("marker1", int_marker) );

  ASSERT_TRUE(server.erase("marker1"));
  ASSERT_FALSE(server.get("marker1", int_marker) );

  server.applyChanges();
  ASSERT_FALSE(server.get("marker1", int_marker) );


  // insert, erase, apply
  server.insert(int_marker);
  ASSERT_TRUE(server.get("marker1", int_marker) );

  ASSERT_TRUE(server.erase("marker1"));
  ASSERT_FALSE(server.get("marker1", int_marker) );

  server.applyChanges();
  ASSERT_FALSE(server.get("marker1", int_marker) );

  // insert, apply, clear, apply
  server.insert(int_marker);
  ASSERT_TRUE(server.get("marker1", int_marker) );

  server.applyChanges();
  ASSERT_TRUE(server.get("marker1", int_marker) );

  server.clear();
  ASSERT_FALSE(server.get("marker1", int_marker) );

  server.applyChanges();
  ASSERT_FALSE(server.get("marker1", int_marker) );

  // insert, setPose, apply, clear, apply
  server.insert(int_marker);
  ASSERT_TRUE(server.setPose("marker1", pose) );

  server.applyChanges();
  server.clear();
  ASSERT_FALSE(server.get("marker1", int_marker) );

  server.applyChanges();

  // erase unknown marker
  ASSERT_FALSE( server.erase("marker1") );

  // avoid subscriber destruction warning
  usleep(1000);
}


// Run all the tests that were declared with TEST()
int main(int argc, char ** argv)
{
  ros::init(argc, argv, "im_server_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
