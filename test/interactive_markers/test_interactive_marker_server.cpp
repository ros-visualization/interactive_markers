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

#include "interactive_marker_fixtures.hpp"
#include "mock_interactive_marker_client.hpp"
#include "timed_expect.hpp"

#include "interactive_markers/interactive_marker_server.hpp"

TEST(TestInteractiveMarkerServer, construction_and_destruction)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("test_interactive_marker_server_node", "");
  {
    interactive_markers::InteractiveMarkerServer server("", node);
  }
  {
    interactive_markers::InteractiveMarkerServer server("test_namespace", node, "test_server_id");
  }
  {
    interactive_markers::InteractiveMarkerServer server(
      "test_namespace",
      node->get_node_base_interface(),
      node->get_node_clock_interface(),
      node->get_node_logging_interface(),
      node->get_node_timers_interface(),
      node->get_node_topics_interface(),
      node->get_node_services_interface(),
      "test_server_id");
  }

  rclcpp::shutdown();
}

TEST(TestInteractiveMarkerServer, insert)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("test_interactive_marker_server_node", "");
  interactive_markers::InteractiveMarkerServer server("test_name", node, "test_server_id");

  // Insert some markers
  std::vector<visualization_msgs::msg::InteractiveMarker> markers = get_interactive_markers();
  for (const auto marker : markers) {
    server.insert(marker);
  }

  // Expect empty until 'applyChanges' is called
  EXPECT_TRUE(server.empty());
  EXPECT_EQ(server.size(), 0u);
  server.applyChanges();
  EXPECT_FALSE(server.empty());
  EXPECT_EQ(server.size(), markers.size());

  rclcpp::shutdown();
}

class TestInteractiveMarkerServerWithMarkers : public ::testing::Test
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
    const std::string topic_namespace = "test_namespace";
    node_ = std::make_shared<rclcpp::Node>("test_interactive_marker_server_node", "");

    server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(
      topic_namespace, node_, "test_server_id");

    // Insert some markers
    markers_ = get_interactive_markers();
    for (const auto marker : markers_) {
      server_->insert(marker);
    }
    server_->applyChanges();

    mock_client_ = std::make_shared<MockInteractiveMarkerClient>(topic_namespace);

    executor_.add_node(mock_client_);
    executor_.add_node(node_);
  }

  void TearDown()
  {
    std::cout << "TearDown()" << std::endl;
    mock_client_.reset();
    server_.reset();
    node_.reset();
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<interactive_markers::InteractiveMarkerServer> server_;
  std::shared_ptr<MockInteractiveMarkerClient> mock_client_;
  std::vector<visualization_msgs::msg::InteractiveMarker> markers_;
};  // class TestInteractiveMarkerServer

TEST_F(TestInteractiveMarkerServerWithMarkers, erase)
{
  // Erase a marker
  EXPECT_TRUE(server_->erase(markers_[0].name));
  EXPECT_EQ(server_->size(), markers_.size());
  server_->applyChanges();
  EXPECT_EQ(server_->size(), markers_.size() - 1u);

  // Erase a marker that has just been inserted
  server_->insert(markers_[0]);
  EXPECT_TRUE(server_->erase(markers_[0].name));
  EXPECT_EQ(server_->size(), markers_.size() - 1u);
  server_->applyChanges();
  EXPECT_EQ(server_->size(), markers_.size() - 1u);

  // Erase an invalid marker
  EXPECT_FALSE(server_->erase("this_Is_the_name_0f_a_marker_that_doesn't ex1st"));
  server_->applyChanges();
  EXPECT_EQ(server_->size(), markers_.size() - 1u);
}

TEST_F(TestInteractiveMarkerServerWithMarkers, clear)
{
  // Clear all markers
  server_->clear();
  EXPECT_EQ(server_->size(), markers_.size());
  server_->applyChanges();
  ASSERT_EQ(server_->size(), 0u);

  // Clear an empty server
  server_->clear();
  EXPECT_EQ(server_->size(), 0u);
}

TEST_F(TestInteractiveMarkerServerWithMarkers, get_marker_by_name)
{
  // Get markers
  for (const auto input_marker : markers_) {
    visualization_msgs::msg::InteractiveMarker output_marker;
    EXPECT_TRUE(server_->get(input_marker.name, output_marker));
    EXPECT_EQ(output_marker.header.frame_id, input_marker.header.frame_id);
    EXPECT_EQ(output_marker.pose.position.x, input_marker.pose.position.x);
    EXPECT_EQ(output_marker.pose.orientation.w, input_marker.pose.orientation.w);
    EXPECT_EQ(output_marker.name, input_marker.name);
    EXPECT_EQ(output_marker.description, input_marker.description);
    ASSERT_EQ(output_marker.menu_entries.size(), input_marker.menu_entries.size());
    for (std::size_t i = 0u; i < output_marker.menu_entries.size(); ++i) {
      EXPECT_EQ(output_marker.menu_entries[i].id, input_marker.menu_entries[i].id);
      EXPECT_EQ(output_marker.menu_entries[i].title, input_marker.menu_entries[i].title);
      EXPECT_EQ(output_marker.menu_entries[i].command, input_marker.menu_entries[i].command);
    }
    ASSERT_EQ(output_marker.controls.size(), input_marker.controls.size());
    for (std::size_t i = 0u; i < output_marker.controls.size(); ++i) {
      EXPECT_EQ(output_marker.controls[i].name, input_marker.controls[i].name);
      EXPECT_EQ(output_marker.controls[i].always_visible, input_marker.controls[i].always_visible);
    }
  }

  // Get an invalid marker
  {
    visualization_msgs::msg::InteractiveMarker output_marker;
    EXPECT_FALSE(server_->get("n0t_the_name_of_@ marker", output_marker));
  }

  // Get a pending erased marker
  {
    EXPECT_TRUE(server_->erase(markers_[0].name));
    visualization_msgs::msg::InteractiveMarker output_marker;
    EXPECT_FALSE(server_->get(markers_[0].name, output_marker));
  }
}

TEST_F(TestInteractiveMarkerServerWithMarkers, set_pose)
{
  // Set a pose
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = 1.0;
    pose.position.y = -2.0;
    pose.position.z = 3.14;
    pose.orientation.w = 0.5;
    EXPECT_TRUE(server_->setPose(markers_[0].name, pose));
    server_->applyChanges();
    visualization_msgs::msg::InteractiveMarker output_marker;
    EXPECT_TRUE(server_->get(markers_[0].name, output_marker));
    EXPECT_EQ(output_marker.pose.position.x, pose.position.x);
    EXPECT_EQ(output_marker.pose.position.y, pose.position.y);
    EXPECT_EQ(output_marker.pose.position.z, pose.position.z);
    EXPECT_EQ(output_marker.pose.orientation.w, pose.orientation.w);
    EXPECT_EQ(output_marker.header.frame_id, markers_[0].header.frame_id);
  }

  // Set a pose with header update
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = 1.0;
    pose.position.y = -2.0;
    pose.position.z = 3.14;
    pose.orientation.w = 0.5;
    std_msgs::msg::Header header;
    header.frame_id = "test_updating_to_a_new_header";
    EXPECT_TRUE(server_->setPose(markers_[0].name, pose, header));
    server_->applyChanges();
    visualization_msgs::msg::InteractiveMarker output_marker;
    EXPECT_TRUE(server_->get(markers_[0].name, output_marker));
    EXPECT_EQ(output_marker.pose.position.x, pose.position.x);
    EXPECT_EQ(output_marker.pose.position.y, pose.position.y);
    EXPECT_EQ(output_marker.pose.position.z, pose.position.z);
    EXPECT_EQ(output_marker.pose.orientation.w, pose.orientation.w);
    EXPECT_NE(output_marker.header.frame_id, markers_[0].header.frame_id);
    EXPECT_EQ(output_marker.header.frame_id, header.frame_id);
  }

  // Set pose of invalid marker
  {
    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0;
    EXPECT_FALSE(server_->setPose("test_n0t_a_valid_marker_n@me", pose));
  }
}

TEST_F(TestInteractiveMarkerServerWithMarkers, set_callback)
{
  EXPECT_TRUE(server_->setCallback(markers_[0].name, nullptr));
  EXPECT_FALSE(server_->setCallback("test_n0t_a_valid_marker_n@me", nullptr));
}

TEST_F(TestInteractiveMarkerServerWithMarkers, feedback_communication)
{
  using namespace std::chrono_literals;

  // Register a callback function to capture output
  visualization_msgs::msg::InteractiveMarkerFeedback output_feedback;
  auto callback =
    [&output_feedback](interactive_markers::InteractiveMarkerServer::FeedbackConstPtr feedback)
    {
      output_feedback = *feedback;
    };
  EXPECT_TRUE(server_->setCallback(markers_[0].name, callback));
  server_->applyChanges();
  // Wait for callback to be registered
  // FIXME(jacobperron): This probably shouldn't be necessary...
  TIMED_EXPECT_EQ(mock_client_->updates_received, 1u, 3s, 10ms, executor_);

  // Populate and publish mock feedback
  visualization_msgs::msg::InteractiveMarkerFeedback feedback;
  feedback.client_id = "test_client_id";
  feedback.marker_name = markers_[0].name;
  feedback.pose.position.x = -3.14;
  feedback.pose.orientation.w = 1.0;
  mock_client_->publishFeedback(feedback);

  // FIXME(jacobperron): Test fails due to rclcpp Time exception
  //                     "can't subtract times with difference time sources"
  // TIMED_EXPECT_EQ(output_feedback.client_id, feedback.client_id, 3s, 10ms, executor_);
  // std::cout << "D" << std::endl;
  // EXPECT_EQ(output_feedback.marker_name, markers_[0].name);
  // EXPECT_EQ(output_feedback.pose.position.x, feedback.pose.position.x);
  // EXPECT_EQ(output_feedback.pose.orientation.w, feedback.pose.orientation.w);
}

TEST_F(TestInteractiveMarkerServerWithMarkers, update_communication)
{
  using namespace std::chrono_literals;

  EXPECT_EQ(mock_client_->updates_received, 0u);
  // This should trigger an update publication
  server_->applyChanges();
  TIMED_EXPECT_EQ(mock_client_->updates_received, 1u, 3s, 10ms, executor_);
  EXPECT_EQ(mock_client_->last_update_message->markers.size(), 0u);
  EXPECT_EQ(mock_client_->last_update_message->poses.size(), 0u);
  EXPECT_EQ(mock_client_->last_update_message->erases.size(), 0u);

  // Erase a marker and update
  ASSERT_TRUE(server_->erase(markers_[0].name));
  server_->applyChanges();
  TIMED_EXPECT_EQ(mock_client_->updates_received, 2u, 3s, 10ms, executor_);
  EXPECT_EQ(mock_client_->last_update_message->markers.size(), 0u);
  EXPECT_EQ(mock_client_->last_update_message->poses.size(), 0u);
  ASSERT_EQ(mock_client_->last_update_message->erases.size(), 1u);
  EXPECT_EQ(mock_client_->last_update_message->erases[0], markers_[0].name);
}

// FIXME(jacobperron): The service response is not received.
/*
TEST_F(TestInteractiveMarkerServerWithMarkers, get_interactive_markers_communication)
{
  using namespace std::chrono_literals;

  MockInteractiveMarkerClient::SharedFuture future = mock_client_->requestInteractiveMarkers();
  auto ret = executor_.spin_until_future_complete<MockInteractiveMarkerClient::SharedResponse>(
    future, 3000ms);
  ASSERT_EQ(ret, rclcpp::executor::FutureReturnCode::SUCCESS);
  visualization_msgs::srv::GetInteractiveMarkers::Response::SharedPtr response = future.get();
  ASSERT_EQ(response->markers.size(), markers_.size());
  for (std::size_t i = 0u; i < markers_.size(); ++i) {
    EXPECT_EQ(response->markers[i].header.frame_id, markers_[i].header.frame_id);
    EXPECT_EQ(response->markers[i].pose.position.x, markers_[i].pose.position.x);
    EXPECT_EQ(response->markers[i].pose.orientation.w, marker_[i].pose.orientation.w);
    EXPECT_EQ(response->markers[i].name, markers_[i].name);
    EXPECT_EQ(response->markers[i].description, markers_[i].description);
    ASSERT_EQ(response->markers[i].menu_entries.size(), markers_[i].menu_entries.size());
    for (std::size_t j = 0u; j < markers_[i].menu_entries.size(); ++j) {
      EXPECT_EQ(response->markers[i].menu_entries[j].id, markers_[i].menu_entries[j].id);
      EXPECT_EQ(response->markers[i].menu_entries[j].title, markers_[i].menu_entries[j].title);
      EXPECT_EQ(response->markers[i].menu_entries[j].command, markers_[i].menu_entries[j].command);
    }
    ASSERT_EQ(response->markers[i].controls.size(), markers_[i].controls.size());
    for (std::size_t j = 0u; j < markers_[i].controls.size(); ++j) {
      EXPECT_EQ(response->markers[i].controls[j].name, markers_[i].controls[j].name);
      EXPECT_EQ(
        response->markers[i].controls[j].always_visible, markers_[i].controls[j].always_visible);
    }
  }
}
*/
