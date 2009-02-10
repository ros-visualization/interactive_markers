///////////////////////////////////////////////////////////////////////////////
// This file is a skeleton showing how to use the navigation primitives to code
// up cooler nav visualizations and interfaces.
//
// Copyright (C) 2008, Morgan Quigley
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice, 
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright 
//     notice, this list of conditions and the following disclaimer in the 
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Stanford University nor the names of its 
//     contributors may be used to endorse or promote products derived from 
//     this software without specific prior written permission.
//   
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
// POSSIBILITY OF SUCH DAMAGE.

#include <ros/node.h>
#include <ros/time.h>
#include <deprecated_msgs/RobotBase2DOdom.h>
#include <robot_msgs/Planner2DGoal.h>

deprecated_msgs::RobotBase2DOdom g_pose;

void pose_updated()
{
  printf("pose: %f, %f, %f\n", g_pose.pos.x, g_pose.pos.y, g_pose.pos.th);
}

void send_goal(double x, double y, double th)
{
  printf("sending %f, %f, %f\n", x, y, th);
  robot_msgs::Planner2DGoal goal;
  goal.goal.x = x;
  goal.goal.y = y;
  goal.goal.th = th;
  goal.enable = 1;
  ros::Node::instance()->publish("goal", goal);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  ros::Node n("multitouch_nav");
  n.subscribe("localizedpose", g_pose, pose_updated, 10);
  n.advertise<robot_msgs::Planner2DGoal>("goal", 10);
  ros::Duration(2).sleep();
  send_goal(42, 20, 2);
  ros::Duration(5).sleep();
  send_goal(40, 20, 2);
  n.spin();
  
  return 0;
}

