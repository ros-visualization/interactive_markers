%module wx_camera_panel
%include "std_string.i"

%{
#include "wx/wxPython/wxPython.h"
#include "wx/wxPython/pyclasses.h"
#include "CameraPanel.h"
%}

%include typemaps.i
%include my_typemaps.i

%import core.i
%import windows.i

%include CameraPanelsGenerated.i

%pythonAppend CameraPanel "self._setOORInfo(self)"

%include "CameraPanel.h"

%init %{

%}