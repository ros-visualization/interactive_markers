#include <wx/wx.h>

class Starter : public wxApp
{
public:
    Starter();
    virtual ~Starter();
    virtual bool OnInit();
};

DECLARE_APP(Starter)
