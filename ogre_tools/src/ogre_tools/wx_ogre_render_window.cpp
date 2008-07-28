#include "wx_ogre_render_window.h"

#include "Ogre.h"

#ifdef __WXGTK__
#include <gdk/gdk.h>
#include <gtk/gtk.h>
#include <gdk/gdkx.h>
#include <wx/gtk/win_gtk.h>
#include <GL/glx.h>
#endif

namespace ogre_tools
{

IMPLEMENT_CLASS (wxOgreRenderWindow, wxControl)

BEGIN_EVENT_TABLE (wxOgreRenderWindow, wxControl)
#ifndef __WXMSW__
    EVT_PAINT (wxOgreRenderWindow::OnPaint)
#endif
    EVT_SIZE (wxOgreRenderWindow::OnSize)
    EVT_MOUSE_EVENTS (wxOgreRenderWindow::OnMouseEvents)
END_EVENT_TABLE ()

//------------------------------------------------------------------------------
unsigned int wxOgreRenderWindow::sm_NextRenderWindowId = 1;
//------------------------------------------------------------------------------
wxOgreRenderWindow::wxOgreRenderWindow (Ogre::Root* ogreRoot, wxWindow *parent, wxWindowID id,
                const wxPoint &pos, const wxSize &size, long style, const wxValidator &validator) 
: wxControl( parent, id, pos, size, style, validator )
, m_RenderWindow( 0 )
{
    m_OgreRoot = ogreRoot;
    
    SetBackgroundStyle(wxBG_STYLE_CUSTOM);
    
    CreateRenderWindow();
}

//------------------------------------------------------------------------------
wxOgreRenderWindow::~wxOgreRenderWindow () 
{
    if (m_RenderWindow)
        m_OgreRoot->detachRenderTarget(m_RenderWindow);

    m_RenderWindow = 0;
}

//------------------------------------------------------------------------------
inline wxSize wxOgreRenderWindow::DoGetBestSize () const 
{
    return wxSize (320, 240);
}
//------------------------------------------------------------------------------
Ogre::RenderWindow *wxOgreRenderWindow::GetRenderWindow () const 
{
    return m_RenderWindow;
}

//------------------------------------------------------------------------------
void wxOgreRenderWindow::OnPaint (wxPaintEvent &evt) 
{
}

//------------------------------------------------------------------------------
void wxOgreRenderWindow::OnSize (wxSizeEvent &evt) 
{
    if (m_RenderWindow) 
    {
        // Setting new size;
        int width;
        int height;
        wxSize size = evt.GetSize ();
        width = size.GetWidth ();
        height = size.GetHeight ();

        m_RenderWindow->resize (width, height);
        // Letting Ogre know the window has been resized;
        m_RenderWindow->windowMovedOrResized ();
    }
}
//------------------------------------------------------------------------------
void wxOgreRenderWindow::OnMouseEvents (wxMouseEvent &evt) 
{

}
//------------------------------------------------------------------------------
void wxOgreRenderWindow::CreateRenderWindow () 
{
    Ogre::NameValuePairList params;
    params["externalWindowHandle"] = GetOgreHandle ();

    // Get wx control window size
    int width;
    int height;
    GetSize (&width, &height);
    // Create the render window
    m_RenderWindow = Ogre::Root::getSingleton ().createRenderWindow (
                    Ogre::String ("OgreRenderWindow") + Ogre::StringConverter::toString (sm_NextRenderWindowId++),
                    width, height, false, &params);

    m_RenderWindow->setActive (true);
}
//------------------------------------------------------------------------------
std::string wxOgreRenderWindow::GetOgreHandle () const 
{
    Ogre::String handle;

#ifdef __WXMSW__
    // Handle for Windows systems
    handle = Ogre::StringConverter::toString((size_t)((HWND)GetHandle()));
#elif defined(__WXGTK__)
    // Handle for GTK-based systems

    GtkWidget *widget = m_wxwindow;
    gtk_widget_set_double_buffered (widget, FALSE);
  gtk_widget_realize( widget );

  // Grab the window object
  GdkWindow *gdkWin = GTK_PIZZA (widget)->bin_window;
  Display* display = GDK_WINDOW_XDISPLAY(gdkWin);
  Window wid = GDK_WINDOW_XWINDOW(gdkWin);

  std::stringstream str;

  // Display
  str << (unsigned long)display << ':';

  // Screen (returns "display.screen")
  std::string screenStr = DisplayString(display);
  std::string::size_type dotPos = screenStr.find(".");
  screenStr = screenStr.substr(dotPos+1, screenStr.size());
  str << screenStr << ':';

  // XID
  str << wid << ':';

  // Retrieve XVisualInfo
  int attrlist[] = { GLX_RGBA, GLX_DOUBLEBUFFER, GLX_DEPTH_SIZE, 16, GLX_STENCIL_SIZE, 8, None };
  XVisualInfo* vi = glXChooseVisual(display, DefaultScreen(display), attrlist);
  str << (unsigned long)vi;

  handle = str.str();
#else
    // Any other unsupported system
    #error Not supported on this platform.
#endif

    return handle;
}

} // namespace ogre_tools
