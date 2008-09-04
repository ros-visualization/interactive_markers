%{
#include "CameraPanelsGenerated.h"
%}

%include typemaps.i
%include my_typemaps.i

%import core.i
%import windows.i

%pythonAppend CameraPanelBase "self._setOORInfo(self)"

class CameraPanelBase : public wxPanel
{
public:
  CameraPanelBase( wxWindow* parent, wxWindowID id = wxID_ANY, const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 487,374 ), long style = wxTAB_TRAVERSAL );
  ~CameraPanelBase();
};

%init %{

%}
