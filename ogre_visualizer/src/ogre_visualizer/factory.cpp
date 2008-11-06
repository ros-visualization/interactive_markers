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

#include "factory.h"

#include "visualization_manager.h"
#include "visualizers/grid_visualizer.h"
#include "visualizers/axes_visualizer.h"
#include "visualizers/point_cloud_visualizer.h"
#include "visualizers/laser_scan_visualizer.h"
#include "visualizers/robot_model_visualizer.h"
#include "visualizers/marker_visualizer.h"
#include "visualizers/octree_visualizer.h"
#include "visualizers/planning_visualizer.h"
#include "visualizers/robot_base2d_pose_visualizer.h"
#include "visualizers/particle_cloud_2d_visualizer.h"
#include "visualizers/poly_line_2d_visualizer.h"
#include "visualizers/map_visualizer.h"
#include "visualizers/tf_visualizer.h"

namespace ogre_vis
{

void registerFactories(VisualizationManager* manager)
{
  manager->registerFactory( GridVisualizer::getTypeStatic(), GridVisualizer::getDescription(), new VisualizerFactoryImpl<GridVisualizer>() );
  manager->registerFactory( AxesVisualizer::getTypeStatic(), AxesVisualizer::getDescription(), new VisualizerFactoryImpl<AxesVisualizer>() );
  manager->registerFactory( PointCloudVisualizer::getTypeStatic(), PointCloudVisualizer::getDescription(), new VisualizerFactoryImpl<PointCloudVisualizer>() );
  manager->registerFactory( LaserScanVisualizer::getTypeStatic(), LaserScanVisualizer::getDescription(), new VisualizerFactoryImpl<LaserScanVisualizer>() );
  manager->registerFactory( RobotModelVisualizer::getTypeStatic(), RobotModelVisualizer::getDescription(), new VisualizerFactoryImpl<RobotModelVisualizer>() );
  manager->registerFactory( MarkerVisualizer::getTypeStatic(), MarkerVisualizer::getDescription(), new VisualizerFactoryImpl<MarkerVisualizer>() );
  manager->registerFactory( OctreeVisualizer::getTypeStatic(), OctreeVisualizer::getDescription(), new VisualizerFactoryImpl<OctreeVisualizer>() );
  manager->registerFactory( PlanningVisualizer::getTypeStatic(), PlanningVisualizer::getDescription(), new VisualizerFactoryImpl<PlanningVisualizer>() );
  manager->registerFactory( RobotBase2DPoseVisualizer::getTypeStatic(), RobotBase2DPoseVisualizer::getDescription(), new VisualizerFactoryImpl<RobotBase2DPoseVisualizer>() );
  manager->registerFactory( ParticleCloud2DVisualizer::getTypeStatic(), ParticleCloud2DVisualizer::getDescription(), new VisualizerFactoryImpl<ParticleCloud2DVisualizer>() );
  manager->registerFactory( PolyLine2DVisualizer::getTypeStatic(), PolyLine2DVisualizer::getDescription(), new VisualizerFactoryImpl<PolyLine2DVisualizer>() );
  manager->registerFactory( MapVisualizer::getTypeStatic(), MapVisualizer::getDescription(), new VisualizerFactoryImpl<MapVisualizer>() );
  manager->registerFactory( TFVisualizer::getTypeStatic(), TFVisualizer::getDescription(), new VisualizerFactoryImpl<TFVisualizer>() );
}

} //namespace ogre_vis
