#ifndef WX_OGRE_RENDER_WINDOW_OGRE_RENDER_WINDOW_H_
#define WX_OGRE_RENDER_WINDOW_OGRE_RENDER_WINDOW_H_

#include "wx/wx.h"

namespace Ogre
{
    class Root;
    class RenderWindow;
}

/** wxWidgets Ogre render window widget.
	Strongly based on the existing wxOgre widget implementation, this one
	isolates the wx component from Ogre, acting as a simple bind to render
	inside a wxWidgets window.

	@author Jesús Alonso Abad 'Kencho', Other contributors (original wxOgre).  Heavily modified by Josh Faust
 */
class wxOgreRenderWindow : public wxControl {
	DECLARE_CLASS (wxOgreRenderWindow)
	DECLARE_EVENT_TABLE ()

// Attributes ------------------------------------------------------------------
	protected:
		/// This control's own render window reference.
		Ogre::RenderWindow* m_RenderWindow;
        
        /// This control's pointer to the global Ogre::Root
        Ogre::Root* m_OgreRoot;

		/// The Id of the next render window
		static unsigned int sm_NextRenderWindowId;

// Methods ---------------------------------------------------------------------
	public:
		/** wx-like Constructor.
			@param parent The parent wxWindow component.
			@param id The control id.
			@param pos The default position.
			@param size The default size.
			@param style The default style for this component.
			@param validator A default validator for the component.
		 */
		wxOgreRenderWindow (Ogre::Root* ogreRoot, wxWindow *parent, wxWindowID id = wxID_ANY,
				const wxPoint &pos = wxDefaultPosition, const wxSize &size = wxDefaultSize,
				long style = wxSUNKEN_BORDER, const wxValidator &validator = wxDefaultValidator);

		/** Virtual destructor.
		 */
		virtual ~wxOgreRenderWindow ();

		/** Overrides the default implementation.
			This override is here for convenience. Returns a symbolic 320x240px size.
			@return A size of 320x240 (just a symbolic 4:3 size).
		 */
		virtual wxSize DoGetBestSize () const;

		/** Gets the associated Ogre render window.
			@return The render window used to paint this control.
		 */
		Ogre::RenderWindow *GetRenderWindow () const;

		/** Painting event callback.
			@param evt Data regarding the painting event.
		 */
		virtual void OnPaint (wxPaintEvent &evt);

		/** Resizing events callback.
			@param evt Data regarding the resize event.
		 */
		virtual void OnSize (wxSizeEvent &evt);

		/** Mouse events callback.
			@remarks Note this will call the specified callback function to process
				the event.
			@param evt Data regarding the mouse event.
		 */
		virtual void OnMouseEvents (wxMouseEvent &evt);

	protected:
		/** Creates an Ogre render window for this widget.
		 */
		virtual void CreateRenderWindow ();

		/** Gets the handle for the render window.
			@return The render window handle.
		 */
		virtual std::string GetOgreHandle () const;

};


#endif /*OGRE_RENDER_WINDOW_H_*/
