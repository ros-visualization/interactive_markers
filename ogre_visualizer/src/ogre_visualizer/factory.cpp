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

namespace ogre_vis
{

void registerFactories(VisualizationManager* manager)
{
  manager->registerFactory( GridVisualizer::getTypeStatic(), new VisualizerFactoryImpl<GridVisualizer>() );
  manager->registerFactory( AxesVisualizer::getTypeStatic(), new VisualizerFactoryImpl<AxesVisualizer>() );
  manager->registerFactory( PointCloudVisualizer::getTypeStatic(), new VisualizerFactoryImpl<PointCloudVisualizer>() );
  manager->registerFactory( LaserScanVisualizer::getTypeStatic(), new VisualizerFactoryImpl<LaserScanVisualizer>() );
  manager->registerFactory( RobotModelVisualizer::getTypeStatic(), new VisualizerFactoryImpl<RobotModelVisualizer>() );
  manager->registerFactory( MarkerVisualizer::getTypeStatic(), new VisualizerFactoryImpl<MarkerVisualizer>() );
  manager->registerFactory( OctreeVisualizer::getTypeStatic(), new VisualizerFactoryImpl<OctreeVisualizer>() );
  manager->registerFactory( PlanningVisualizer::getTypeStatic(), new VisualizerFactoryImpl<PlanningVisualizer>() );
}

} //namespace ogre_vis
