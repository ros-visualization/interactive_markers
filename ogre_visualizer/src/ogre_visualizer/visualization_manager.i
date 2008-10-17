%{
#include "visualization_manager.h"

#include "visualizers/axes_visualizer.h"
#include "visualizers/grid_visualizer.h"
#include "visualizers/laser_scan_visualizer.h"
#include "visualizers/marker_visualizer.h"
#include "visualizers/octree_visualizer.h"
#include "visualizers/planning_visualizer.h"
#include "visualizers/point_cloud_visualizer.h"
#include "visualizers/robot_model_visualizer.h"
#include "visualizers/robot_base2d_pose_visualizer.h"
%}

%include std_string.i

%include "visualizers/axes_visualizer.i"
%include "visualizers/grid_visualizer.i"
%include "visualizers/laser_scan_visualizer.i"
%include "visualizers/marker_visualizer.i"
%include "visualizers/octree_visualizer.i"
%include "visualizers/planning_visualizer.i"
%include "visualizers/point_cloud_visualizer.i"
%include "visualizers/robot_model_visualizer.i"
%include "visualizers/robot_base2d_pose_visualizer.i"

%pythonAppend VisualizationManager "self._setOORInfo(self)"

%include "visualization_manager.h"

%extend ogre_vis::VisualizationManager
{
  %template(createAxesVisualizer) createVisualizer<ogre_vis::AxesVisualizer>;
  %template(createGridVisualizer) createVisualizer<ogre_vis::GridVisualizer>;
  %template(createLaserScanVisualizer) createVisualizer<ogre_vis::LaserScanVisualizer>;
  %template(createMarkerVisualizer) createVisualizer<ogre_vis::MarkerVisualizer>;
  %template(createOctreeVisualizer) createVisualizer<ogre_vis::OctreeVisualizer>;
  %template(createPlanningVisualizer) createVisualizer<ogre_vis::PlanningVisualizer>;
  %template(createPointCloudVisualizer) createVisualizer<ogre_vis::PointCloudVisualizer>;
  %template(createRobotModelVisualizer) createVisualizer<ogre_vis::RobotModelVisualizer>;
  %template(createRobotBase2DPoseVisualizer) createVisualizer<ogre_vis::RobotBase2DPoseVisualizer>;
};

%init %{

%}

