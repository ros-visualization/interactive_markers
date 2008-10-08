%module ogre_visualizer
%include "std_string.i"
%include "std_vector.i"

%{
#include "wx/wxPython/wxPython.h"
#include "wx/wxPython/pyclasses.h"
%}

%include typemaps.i
%include my_typemaps.i

%import core.i
%import windows.i

%include "helpers/color.i"
%include "visualizer_base.i"
%include "visualization_panel.i"

%include "visualizers/axes_visualizer.i"
%include "visualizers/grid_visualizer.i"
%include "visualizers/laser_scan_visualizer.i"
%include "visualizers/marker_visualizer.i"
%include "visualizers/octree_visualizer.i"
%include "visualizers/planning_visualizer.i"
%include "visualizers/point_cloud_visualizer.i"
%include "visualizers/robot_model_visualizer.i"

%include "helpers/color.i"

%init %{

%}
