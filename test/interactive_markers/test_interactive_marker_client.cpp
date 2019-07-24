// Copyright (c) 2019, Open Source Robotics Foundation, Inc.
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
//  * Neither the name of Open Source Robotics Foundation, Inc. nor the names of its
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
#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "tf2/buffer_core.h"

#include "interactive_marker_fixtures.hpp"
#include "mock_interactive_marker_server.hpp"
#include "timed_expect.hpp"

#include "interactive_markers/interactive_marker_client.hpp"

using ClientState = interactive_markers::InteractiveMarkerClient::StateT;

TEST(TestInteractiveMarkerClientInitialize, construction_destruction)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("test_interactive_marker_client_node", "");
  auto buffer = std::make_shared<tf2::BufferCore>();

  {
    interactive_markers::InteractiveMarkerClient client(node, buffer);
  }
  {
    interactive_markers::InteractiveMarkerClient client(
      node, buffer, "test_frame_id", "test_namespace");
  }
  {
    interactive_markers::InteractiveMarkerClient client(
      node->get_node_base_interface(),
      node->get_node_topics_interface(),
      node->get_node_services_interface(),
      node->get_node_graph_interface(),
      node->get_node_logging_interface(),
      buffer,
      "test_frame_id",
      "test_namespace");
  }

  rclcpp::shutdown();
}

class TestInteractiveMarkerClient : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    std::cout << "SetUpTestCase()" << std::endl;
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    std::cout << "TearDownTestCase()" << std::endl;
    rclcpp::shutdown();
  }

  void SetUp()
  {
    std::cout << "SetUp()" << std::endl;
    topic_namespace_ = "test_namespace";
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    node_ = std::make_shared<rclcpp::Node>("test_interactive_marker_server_node", "");
    buffer_ = std::make_shared<tf2::BufferCore>();
    client_ = std::make_unique<interactive_markers::InteractiveMarkerClient>(
      node_, buffer_, "test_frame_id", topic_namespace_);
    executor_->add_node(node_);
  }

  void TearDown()
  {
    std::cout << "TearDown()" << std::endl;
    client_.reset();
    buffer_.reset();
    node_.reset();
    executor_.reset();
  }

  std::string topic_namespace_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2::BufferCore> buffer_;
  std::unique_ptr<interactive_markers::InteractiveMarkerClient> client_;
};  // class TestInteractiveMarkerClient

TEST_F(TestInteractiveMarkerClient, states)
{
  using namespace std::chrono_literals;

  // Not using member client for this test
  client_.reset();

  // IDLE -> IDLE
  {
    interactive_markers::InteractiveMarkerClient client(node_, buffer_);
    EXPECT_EQ(client.getState(), ClientState::IDLE);
    client.shutdown();
    EXPECT_EQ(client.getState(), ClientState::IDLE);
  }

  // INIT -> IDLE
  {
    interactive_markers::InteractiveMarkerClient client(
      node_, buffer_, "test_frame_id", "test_namespace");
    EXPECT_EQ(client.getState(), ClientState::INIT);
    client.shutdown();
    EXPECT_EQ(client.getState(), ClientState::IDLE);
  }

  // IDLE -> INIT -> IDLE -> INIT
  {
    interactive_markers::InteractiveMarkerClient client(node_, buffer_);
    EXPECT_EQ(client.getState(), ClientState::IDLE);
    client.subscribe("test_namespace");
    EXPECT_EQ(client.getState(), ClientState::INIT);
    client.shutdown();
    EXPECT_EQ(client.getState(), ClientState::IDLE);
    client.subscribe("test_namespace");
    EXPECT_EQ(client.getState(), ClientState::INIT);
  }

  // INIT -> RUNNING -> IDLE
  {
    auto mock_server = std::make_shared<MockInteractiveMarkerServer>(topic_namespace_);
    interactive_markers::InteractiveMarkerClient client(
      node_, buffer_, "test_frame_id", topic_namespace_);
    EXPECT_EQ(client.getState(), ClientState::INIT);
    executor_->add_node(mock_server);
    // Note, we repeatedly call the client's update method to process the service response
    auto update_func = std::bind(&interactive_markers::InteractiveMarkerClient::update, &client);
    TIMED_EXPECT_EQ(client.getState(), ClientState::RUNNING, 3s, 10ms, (*executor_), update_func);
    client.shutdown();
    EXPECT_EQ(client.getState(), ClientState::IDLE);
  }

  // INIT -> RUNNING -> INIT -> RUNNING
  {
    auto mock_server = std::make_shared<MockInteractiveMarkerServer>(topic_namespace_);
    interactive_markers::InteractiveMarkerClient client(
      node_, buffer_, "test_frame_id", topic_namespace_);
    EXPECT_EQ(client.getState(), ClientState::INIT);
    executor_->add_node(mock_server);
    // Note, we repeatedly call the client's update method to process the service response
    auto update_func = std::bind(&interactive_markers::InteractiveMarkerClient::update, &client);
    TIMED_EXPECT_EQ(client.getState(), ClientState::RUNNING, 3s, 10ms, (*executor_), update_func);
    // Bringing the server offline should prompt a transition back to INIT
    // FIXME(jacobperron): transition RUNNING -> INIT not happening
    // executor_->remove_node(mock_server);
    // mock_server.reset();
    // TIMED_EXPECT_EQ(client.getState(), ClientState::INIT, 3s, 10ms, (*executor_), update_func);
    // Bring the server back online
    // mock_server = std::make_shared<MockInteractiveMarkerServer>(topic_namespace_);
    // executor_->add_node(mock_server);
    // TIMED_EXPECT_EQ(client.getState(), ClientState::RUNNING, 3s, 10ms, *executor_, update_func);
  }

  // INIT -> RUNNING -> INIT -> IDLE
  {
    auto mock_server = std::make_shared<MockInteractiveMarkerServer>(topic_namespace_);
    interactive_markers::InteractiveMarkerClient client(
      node_, buffer_, "test_frame_id", topic_namespace_);
    EXPECT_EQ(client.getState(), ClientState::INIT);
    executor_->add_node(mock_server);
    // Note, we repeatedly call the client's update method to process the service response
    auto update_func = std::bind(&interactive_markers::InteractiveMarkerClient::update, &client);
    TIMED_EXPECT_EQ(client.getState(), ClientState::RUNNING, 3s, 10ms, (*executor_), update_func);
    // Bringing the server offline should prompt a transition back to INIT
    // FIXME(jacobperron): transition RUNNING -> INIT not happening
    // executor_->remove_node(mock_server);
    // mock_server.reset();
    // TIMED_EXPECT_EQ(client.getState(), ClientState::INIT, 3s, 10ms, (*executor_), update_func);
    client.shutdown();
    EXPECT_EQ(client.getState(), ClientState::IDLE);
  }
}

TEST_F(TestInteractiveMarkerClient, init_callback)
{
  using namespace std::chrono_literals;

  bool called = false;
  auto callback = [&called](visualization_msgs::srv::GetInteractiveMarkers::Response::SharedPtr)
    {
      called = true;
    };

  client_->setInitCb(callback);

  // Creating a server should trigger the callback
  auto mock_server = std::make_shared<MockInteractiveMarkerServer>(topic_namespace_);
  executor_->add_node(mock_server);
  // FIXME(jacobperron): Callback not triggered
  // TIMED_EXPECT_EQ(called, true, 3s, 10ms, (*executor_));
}

TEST_F(TestInteractiveMarkerClient, update_callback)
{
  using namespace std::chrono_literals;

  geometry_msgs::msg::Pose output_pose;
  auto callback = [&output_pose](visualization_msgs::msg::InteractiveMarkerUpdate::SharedPtr msg)
    {
      ASSERT_EQ(msg->poses.size(), 1u);
      output_pose = msg->poses[0].pose;
    };

  client_->setUpdateCb(callback);

  // Publish an update from a server
  auto mock_server = std::make_shared<MockInteractiveMarkerServer>(topic_namespace_);
  geometry_msgs::msg::Pose input_pose;
  input_pose.position.x = 42.0;
  input_pose.position.y = -2.2;
  input_pose.position.z = 0.0;
  input_pose.orientation.w = 0.5;
  mock_server->publishUpdate(input_pose);
  executor_->add_node(mock_server);
  auto update_func = std::bind(
    &interactive_markers::InteractiveMarkerClient::update, client_.get());
  // FIXME(jacobperron): Callback not triggered
  // TIMED_EXPECT_EQ(
  //   output_pose.position.x, input_pose.position.x, 3s, 10ms, (*executor_), update_func);
  // EXPECT_EQ(output_pose.position.y, input_pose.position.y);
  // EXPECT_EQ(output_pose.position.z, input_pose.position.z);
  // EXPECT_EQ(output_pose.orientation.w, input_pose.orientation.w);
}
