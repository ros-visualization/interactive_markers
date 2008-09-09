%{
#include "visualizer_base.h"
%}

%include typemaps.i
%include my_typemaps.i

%import core.i
%import windows.i

%pythonAppend VisualizerBase "self._setOORInfo(self)"

%include "visualizer_base.h"

%init %{

%}


