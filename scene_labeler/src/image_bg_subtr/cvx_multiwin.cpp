//-------------------------------------------------------------------------------
//
//    cvx_multiwin.cpp
//    OpenCV based code generating a multi-panel image window.
//    Copyright (C) 2006 Adrian Kaehler
//
//    This program is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program; if not, write to the Free Software
//    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
//
//
//-------------------------------------------------------------------------------
#include "cvx_multiwin.h"

int		CvxMultiWin::M_SPC					= 10;
CvFont*	CvxMultiWin::M_FONT					= NULL;
bool	CvxMultiWin::M_CLASS_INITIALIZED	= false;

// The user is required to make an initialize() call before using
// the multi-window.  This call should indicate the name for the
// window (as per cvNamedWindow()), as well as the width and height
// of the panel images.  c,r specify the number of columns and
// rows (respectively) of panels to create.  depth and channels
// are passed directly to the multi-panel image.
//
bool CvxMultiWin::initialize( 
	const char* name, 
	int w, 
	int h, 
	int Nc, 
	int Nr, 
	int depth, 
	int channels,
	int flags,
	int origin
) {

	// In the case of an initialize call to an already inititalized
	// window, we just do nothing.
	//
	if( m_initialized==true ) return false;

	// A Font object exists which the CvxMultiWin class uses to 
	// write labels (when they are present) on the subwindows.
	//  This Font is a static member of the class, so is only
	// initialized when the class has never been initialized.
	// (This will happen on the first call to CvxMultiWin::initialize()
	// obviously.)
	//
	if( ! M_CLASS_INITIALIZED ) {
		M_FONT = new CvFont;
		assert( M_FONT );
		cvInitFont( M_FONT, CV_FONT_VECTOR0, (float)1.0, (float)1.0, 0.0, 2 );
		M_CLASS_INITIALIZED = true;
	}

	cvNamedWindow( name, flags );
	strncpy( m_szWindowName, name, 256 );

	m_width				= (w+M_SPC)*Nc + M_SPC;
	m_height			= (h+M_SPC)*Nr + M_SPC;
	m_panel_width		= w;
	m_panel_height		= h;
	m_columns			= Nc;
	m_rows				= Nr;
	m_origin			= origin;
//	m_source_img_size.reserve(Nc*Nr); //Set enough space to record CvSize of each source image for each panel
	int i;
	m_source_img_width.assign(Nc*Nr,w);
	m_source_img_height.assign(Nc*Nr,h);
	size_t foo = m_source_img_width.size();
	foo = m_source_img_height.size();
//	for(i=0; i<Nc*Nr; i++)
//	{
//		m_source_img_size.at(i).height = h;  //Default each panel source to original width. Set actual values in paint
//		m_source_img_size[i].width = w;
//	}

	m_implicit_target	= 0;

	if( m_image != NULL ) { cvReleaseImage( &m_image ); }
	m_image = cvCreateImage( cvSize( m_width, m_height ), depth, channels );
	m_image->origin = m_origin; //Added to allow which side up control

	cvAddS( m_image, CV_RGB(128,128,128), m_image );
//	cvAddS( m_image, cvScalar(128,128,128,0), m_image );

	// Every multiwin has a set of
	// labels, one for each panel.  Whenever a panel is drawn, the label
	// will automatically be labeled with whatever label exists for that
	// panel.
	//

	m_labels = new char*[Nr*Nc];
	assert(m_labels);
	for(i=0; i<Nr*Nc; i++ ) { 
		m_labels[i]=new char[256];
		assert( m_labels[i] );
		sprintf( m_labels[i], "" ); 
	}

#if 0
    // create avi recorder
    aviWriter = cvCreateVideoWriter("multiwin.avi", -1, 15.0, 
        cvSize(640, 480), 1);
#endif

	// Make sure that we know hereafter that this multiwin has been 
	// initialized!
	//
	m_initialized		= true;

	return true;
}


// Here the caller gives an image and we will draw it into multiwindow.
// If the caller omits the parameter "r", then the argument "rc" is
// interpreted as the position in raster scan order.  If "r" is supplied,
// then rc is interpreted as the column, and r is the row.
// If 'rc' is not provided either, then the image is drawn to m_implicit_target,
// which is then incremented.  m_implicit_target is cleared on a draw
// command.
//
//	To visualize floating point images, optional values allow you to:
//  min_value_float		- This and anything less will become zero
//	max_value_float		- This and anything greater will become 255
//						All else will be scaled proportionately
//
void CvxMultiWin::paint( 
	IplImage* in, 
	int rc, 
	int r, 
	bool redraw_now,
	float min_value_float,
	float max_value_float
) {

	// we would like to be able to visualize images which are
	// not in 8-bit format.  To do this, we create a special
	// image called 'exotic' which is our special visualization
	// of a particular image type.  The visualization scheme
	// is "case by case", so there is no universal paradigm for
	// deciding how to raw an exotic type.
	//
	static IplImage* exotic=NULL;
	if( in->depth != IPL_DEPTH_8U ) {

		int Nc = in->nChannels;

		if( exotic && (exotic->width!=in->width || exotic->height!=in->height) ) {
			cvReleaseImage( &exotic );
		}
		if( exotic==NULL ) {
			exotic = cvCreateImage( cvGetSize(in), IPL_DEPTH_8U, 3 );
		}

		if( in->depth==IPL_DEPTH_16S )
		{
			double min[3], max[3], scale[3];
			double minF, maxF;

			for(int i = 0; i < Nc; i++)
			{
				cvSetImageCOI(in, i+1);
				cvMinMaxLoc(in, &minF, &maxF);
				min[i] = 0; //minF;
				max[i] = maxF;
				scale[i] = 255.0 / (max[i] - min[i]);
			}

			cvSetImageCOI(in, 0);

			int v;
			double fv;
			for (int y = 0; y < in->height; y++) {
				short* p = (short*)(in->imageData+y*in->widthStep);
				uchar* q = (uchar*)(exotic->imageData+y*exotic->widthStep);
				for( int x=0; x<in->width; ++x ) {
						for( int c=0; c<3; ++c ) {
						if (Nc != 1)
						{
							for( int c=0; c<Nc; c++ )
							{
    							fv = abs(p[Nc*x+c]); //Put values between 0 and 255
								if(isnan(fv)){fv = 0.0;}
	    						else if(fv <= min[c]){fv = min[c];}
								else if(fv >= max[c]) {fv = max[c];}
			    				else { fv = scale[c]*(fv - min[c]);}
				    			v = (int)fv;
          						q[Nc*x+c] = v;
							}
						}
						else
						{
      						fv = abs(p[x]); //Put values between 0 and 255
							if(isnan(fv)){fv = 0.0;}
	    					else if(fv <= min[0]){fv = min[0];}
  		    				else if(fv >= max[0]) {fv = max[0];}
  			    			else { fv = scale[0]*(fv - min[0]);}
  				    		v = (int)fv;
      						q[3*x] = q[3*x+1] = q[3*x+2] = v;
						}
					}
				}
			}
			in = exotic;
		}
		else if( in->depth==IPL_DEPTH_16U )
		{
			double min[3], max[3], scale[3];
			double minF, maxF;

			for(int i = 0; i < Nc; i++)
			{
				cvSetImageCOI(in, i+1);
				cvMinMaxLoc(in, &minF, &maxF);
				min[i] = 0; //minF;
				max[i] = maxF;
				scale[i] = 255.0 / (max[i] - min[i]);
			}

			cvSetImageCOI(in, 0);

			int v;
			double fv;
			for( int y=0; y<in->height; ++y )
			{
				unsigned short* p = (unsigned short*)(in->imageData+y*in->widthStep);
				uchar* q = (uchar*)(exotic->imageData+y*exotic->widthStep);
				for( int x=0; x<in->width; ++x )
				{
					if (Nc != 1)
					{
						for( int c=0; c<Nc; c++ )
						{
    						fv = p[Nc*x+c]; //Put values between 0 and 255
							if(isnan(fv)){fv = 0.0;}
	    					else if(fv <= min[c]){fv = min[c];}
							else if(fv >= max[c]) {fv = max[c];}
			    			else { fv = scale[c]*(fv - min[c]);}
				    		v = (int)fv;
          					q[Nc*x+c] = v;
						}
					}
					else
					{
      					fv = p[x]; //Put values between 0 and 255
						if(isnan(fv)){fv = 0.0;}
	    				else if(fv <= min[0]){fv = min[0];}
  		    			else if(fv >= max[0]) {fv = max[0];}
  			    		else { fv = scale[0]*(fv - min[0]);}
  				    	v = (int)fv;
      					q[3*x] = q[3*x+1] = q[3*x+2] = v;
					}
				}
			}
			in = exotic;
        }
		else if( in->depth == IPL_DEPTH_32F )
		{
			double min[3], max[3], scale[3];
			double minF, maxF;

			for(int i = 0; i < Nc; i++)
			{
				cvSetImageCOI(in, i+1);
				cvMinMaxLoc(in, &minF, &maxF);
				min[i] = minF;
				max[i] = maxF;
				scale[i] = 255.0 / (max[i] - min[i]);
			}

			cvSetImageCOI(in, 0);

			int v;
			double fv;
			for( int y=0; y<in->height; ++y ) {
				float* p = (float*)(in->imageData+y*in->widthStep);
				uchar* q = (uchar*)(exotic->imageData+y*exotic->widthStep);
				for( int x=0; x<in->width; ++x ) {
                    if (Nc != 1) {
					    for( int c=0; c<Nc; c++ ) {
    						fv = p[Nc*x+c]; //Put values between 0 and 255
							if(isnan(fv)){fv = 0.0;}
	    					else if(fv <= min[c]){fv = min[c];}
							else if(fv >= max[c]) {fv = max[c];}
			    			else { fv = scale[c]*(fv - min[c]);}
				    		v = (int)fv;
          					q[Nc*x+c] = v;
                        }
					} else {
      					fv = p[x]; //Put values between 0 and 255
						if(isnan(fv)){fv = 0.0;}
	    				else if(fv <= min[0]){fv = min[0];}
  		    			else if(fv >= max[0]) {fv = max[0];}
  			    		else { fv = scale[0]*(fv - min[0]);}
  				    	v = (int)fv;
      				    q[3*x] = q[3*x+1] = q[3*x+2] = v;
                    }
				}
			}
			in = exotic;
		}
		// other cases would go here for other exotic types...
		// } else if { in->depth==XXX && in->nChannels==YYY ) {
		else
		{
			cvReleaseImage( &exotic );
		}
	}

	if( !m_initialized ) return;

	//Make sure we draw to a legal panel by forcing it within bounds
	int implicit_max;
	if(rc <= -1) //Make sure rc = -1, r > -1 doesn't happen
	{
		rc = -1; 
		r = -1;
		implicit_max = m_rows*m_columns; //Keep implicit target w/in range
		if(m_implicit_target < 0) m_implicit_target = 0;
		else if (m_implicit_target >= implicit_max) m_implicit_target = 0;
	} 
	else if(r <= -1) //rc is set but r is not.  Keep rc in bounds;
	{
		implicit_max = m_rows*m_columns;
		if(implicit_max < 1) implicit_max = 1;
		r = -1;
		if(rc >= implicit_max) rc = implicit_max -1; //Keep rc in range
	}
	else //Keep rows and columns within bounds
	{ 
		if(rc >= m_columns) rc = m_columns -1;
		if(rc < 0) rc = 0;
		if(r >= m_rows) r = m_rows - 1;
		if(r < 0) r = 0;
	}

	// Then, figure out where we are supposed to draw the image 
	// we were given: 
	//
	int c;
	if( rc==-1 ) { rc = m_implicit_target++; }
	if( r!=-1 ) { 
		c = rc;
	} else {
		c = rc % m_columns;
		r = rc / m_columns;
	}
	int r_old = r;
	if(m_origin == IPL_ORIGIN_BL) r = m_rows - 1 - r;

	// next compute the location of the panel into which we will draw:
	//
	int atx = M_SPC + c * (m_panel_width  + M_SPC);
	int aty = M_SPC + r * (m_panel_height + M_SPC);

	//Fill in the actual width and height of this source image in it's panel location
	m_source_img_width[r*m_columns + c] = in->width; //record actual width and height
	m_source_img_height[r*m_columns + c] = in->height; //record actual width and height

	// Now we have to create a new image which is the small form
	// of the input image image.  (Note that 'small' has the same
	// number of channels as 'in'.
	//
	IplImage* mini = cvCreateImage(
		cvSize( m_panel_width, m_panel_height ),
		in->depth,
		in->nChannels
	);
	mini->origin = m_origin; //Keep drawing right side up
	cvResize( in, mini );

	IplImage* panel = cvCreateImage(
		cvSize( m_panel_width, m_panel_height ),
		m_image->depth,
		m_image->nChannels
	);
	panel->origin = m_origin; //Keep drawing right side up

	if( mini->nChannels == 1 ) {
		cvCvtColor( mini, panel, CV_GRAY2BGR );
	} else {
		cvCopy( mini, panel );
	}
	cvReleaseImage( &mini );

	CvRect rect = cvRect( atx, aty, m_panel_width, m_panel_height );
	cvSetImageROI( m_image, rect );

	r = r_old;

	cvPutText( panel, m_labels[r*m_columns+c], cvPoint(5,25), M_FONT, CVX_CYAN );
	cvCopy( panel, m_image );
	cvResetImageROI( m_image );

	cvReleaseImage( &panel );

	if( redraw_now ) {
		redraw();
	}
}
	//MULTI-WINDOW MOUSE SUPPORT  
	// You may use cvSetMouseCallback and set up a mouse call back function
	// with a multiwin.  The call back function has the basic prototype:
	//
	//   void mouseCallBack( int event, int x, int y, int flags,void *foo = NULL )
	// 
	// EXAMPLE:
	// CvxMultiWin mw; //Multi-window 
	// ...
	//			mw.initialize("Histo", image->width, image->height, 2,2,8,3,1,IPL_ORIGIN_BL); // set up 2x2 mw
	//			mw.set_label( 0, 0, "c0 r0" ); // label each panel
	//			mw.set_label(1,0,"c1 r0");
	//			mw.set_label(0,1,"c0 r1");
	//			mw.set_label(1,1,"c1 r1");
	//		    cvSetMouseCallback( "Histo", on_mousemw ); //Make "on_mousemw" the mouse call back funtion
	//
	// A mouse event in the multiwindow will now call on_mousemw with the x,y image (not window) coordinate of the mouse
	// The problem is that x and y refer to the whole window.  There is also the 
	// problem of getting the image origin right.  Below are support functions to
	// translate x & y to to coordinates within the scaled window and to draw or sample
	// from the associated panel source image if you pass a pointer to it.


	//Given a mouse x,y event in the multiwindow, 
	// calculate the panel Col, Row and x,y offset in that panel's coordinates
	// Return: m_origin of the multiwindow, or < 0 => mouse is in panel border region
	//         -1 => not initialized
	// NOTES:
	//  * You might have to adjust panel's to height - y depending on your associated image's origin.
	//  * x and y are in panel coordinates which are possibly rescaled from the image you passed in.
	//    This x and y are for an image of m_panel_width by m_panel_height you may have to rescale to
	//    in order to get the intended x,y target in the associated image.
    //	* Two versions of this algorithm for backward compatability, one with and one without image_x, image_y
	int CvxMultiWin::panelXY(int &x, int &y, int &panelCol, int &panelRow)
	{
		if (!m_initialized) return(-1);
		panelCol = x/(m_panel_width+M_SPC);
		panelRow = y/(m_panel_height+M_SPC);
		if(panelRow >= m_rows) panelRow--;    //the last M_SPC causes Row/Col to step one too far
		if(panelCol >= m_columns) panelCol--;
		x = x - ((m_panel_width+M_SPC)*panelCol) - M_SPC;
		if(x >= m_panel_width) x = -x;
		y = y - ((m_panel_height+M_SPC)*panelRow) - M_SPC;
		if(y >= m_panel_height) y = -y;
		return(m_origin);
	}
	int CvxMultiWin::panelXY(int &x, int &y, int &panelCol, int &panelRow, int &image_x, int &image_y)
	{
		if (!m_initialized) return(-1);
		panelCol = x/(m_panel_width+M_SPC);
		panelRow = y/(m_panel_height+M_SPC);
		if(panelRow >= m_rows) panelRow--;     //the last M_SPC causes Row/Col to step one too far
		if(panelCol >= m_columns) panelCol--;
		x = x - ((m_panel_width+M_SPC)*panelCol) - M_SPC;
		if(x >= m_panel_width) x = -x;
		y = y - ((m_panel_height+M_SPC)*panelRow) - M_SPC;
		if(y >= m_panel_height) y = -y;
		//For backwards compatability, only fill in image_x or _y if they are not defaulted to -1
		if(image_x != -1) //Calculate x in the original source image
			image_x = x*m_source_img_width[panelRow*m_columns + panelCol]/m_panel_width;
		if(image_y != -1) //Calculate x in the original source image
			image_y = y*m_source_img_height[panelRow*m_columns + panelCol]/m_panel_height;
		return(m_origin);
	}

// The following function was a bad idea ...	//Given a mouse x,y, put a pixel there relative to the whole screen
//	void CvxMultiWin::putPixelWholeImage(int x, int y)
//	{
////		int nChannels = m_image->nChannels; //Get number of channels for x offset
//		if ((!m_initialized)||(m_image == NULL)) return;
//
//		char *cptr = &CV_IMAGE_ELEM(m_image,char,y,x);
////		char *cptr = (((char*)((m_image)->imageData + (m_image)->widthStep*(y)))[(x)]);
//		*cptr = 100;
//		cvShowImage( m_szWindowName, m_image );
////		CV_IMAGE_ELEM( image, elemtype, row, col )       \
//    (((elemtype*)((image)->imageData + (image)->widthStep*(row)))[(col)])
//
//if(m_origin == IPL_ORIGIN_TL)
//{
//  int  nChannels;     /* Most of OpenCV functions support 1,2,3 or 4 channels */
//  int  alphaChannel;  /* ignored by OpenCV */
//  int  depth;         /* pixel depth in bits: IPL_DEPTH_8U, IPL_DEPTH_8S, IPL_DEPTH_16S,
//                           IPL_DEPTH_32S, IPL_DEPTH_32F and IPL_DEPTH_64F are supported */

//#define CV_RGB( r, g, b )  cvScalar( (b), (g), (r) )

//typedef struct CvScalar
//{
//    double val[4];
//}
//CvScalar;
//  }



