/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#ifndef OGRE_VISUALIZER_LASER_SCAN_VISUALIZER_H
#define OGRE_VISUALIZER_LASER_SCAN_VISUALIZER_H

#include "../visualizer_base.h"
#include "laser_scan_utils/laser_scan.h"
#include "ogre_tools/point_cloud.h"

#include "std_msgs/LaserScan.h"
#include "std_msgs/PointCloudFloat32.h"
#include "std_msgs/Empty.h"

#include <deque>

namespace ros
{
  class node;
}

class rosTFClient;

namespace ogre_vis
{

// TODO: find out some way to share most of this code with PointCloudVisualizer

class LaserScanVisualizer : public VisualizerBase
{
public:
  enum Style
  {
    Points,
    Billboards,
  };

  LaserScanVisualizer( Ogre::SceneManager* sceneManager, ros::node* node, rosTFClient* tfClient, const std::string& name, bool enabled );
  ~LaserScanVisualizer();

  void SetCloudTopic( const std::string& topic );
  void SetScanTopic( const std::string& topic );

  void SetColor( float r, float g, float b );
  void SetDecayTime( float time ) { point_decay_time_ = time; }
  void SetStyle( Style style );
  void SetBillboardSize( float size );

  virtual void Update( float dt );

  // Overrides from VisualizerBase
  virtual void FillPropertyGrid( wxPropertyGrid* propertyGrid );
  virtual void PropertyChanged( wxPropertyGridEvent& event );

protected:
  virtual void OnEnable();
  virtual void OnDisable();

  void Subscribe();
  void Unsubscribe();

  void TransformCloud();
  void CullPoints();

  void IncomingCloudCallback();
  void IncomingScanCallback();

  ogre_tools::PointCloud* cloud_;

  std::string cloud_topic_;
  std::string scan_topic_;
  std_msgs::PointCloudFloat32 cloud_message_;
  std_msgs::LaserScan scan_message_;

  laser_scan::LaserProjection laser_projection_;

  float r_;
  float g_;
  float b_;

  float intensity_min_;
  float intensity_max_;

  typedef std::vector< ogre_tools::PointCloud::Point > V_Point;
  typedef std::deque< V_Point > DV_Point;
  DV_Point points_;

  typedef std::deque< float > D_float;
  D_float point_times_;
  float point_decay_time_;

  Style style_;
  float billboard_size_;
};

} // namespace ogre_vis

#endif
