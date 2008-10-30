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
#include "visualizers/particle_cloud_2d_visualizer.h"
#include "visualizers/poly_line_2d_visualizer.h"
#include "visualizers/map_visualizer.h"
#include "visualizers/tf_visualizer.h"
%}

%include std_string.i

%include "visualizers/axes_visualizer.h"
%include "visualizers/grid_visualizer.h"
%include "visualizers/laser_scan_visualizer.h"
%include "visualizers/marker_visualizer.h"
%include "visualizers/octree_visualizer.h"
%include "visualizers/planning_visualizer.h"
%include "visualizers/point_cloud_visualizer.h"
%include "visualizers/robot_model_visualizer.h"
%include "visualizers/robot_base2d_pose_visualizer.h"
%include "visualizers/particle_cloud_2d_visualizer.h"
%include "visualizers/poly_line_2d_visualizer.h"
%include "visualizers/map_visualizer.h"
%include "visualizers/tf_visualizer.h"

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
  %template(createParticleCloud2DVisualizer) createVisualizer<ogre_vis::ParticleCloud2DVisualizer>;
  %template(createPolyLine2DVisualizer) createVisualizer<ogre_vis::PolyLine2DVisualizer>;
  %template(createMapVisualizer) createVisualizer<ogre_vis::MapVisualizer>;
  %template(createTFVisualizer) createVisualizer<ogre_vis::TFVisualizer>;
};

%init %{

%}

