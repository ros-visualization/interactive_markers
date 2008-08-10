////////YUV CODEBOOK ////////////////////////////////////////////////////
//---------------------------------------------------------------------------- 
 // 
 //	File name: cv_yuv_codebook.h
 // 
 //----------------------------------------------------------------------------
 //
 // Copyright (c) 2006, Gary Bradski
 // All rights reserved.
 //
 // Redistribution and use in source and binary forms, with or without
 // modification, are permitted provided that the following conditions are met:
 //
 //     * Redistributions of source code must retain the above copyright
 //       notice, this list of conditions and the following disclaimer.
 //     * Redistributions in binary form must reproduce the above copyright
 //       notice, this list of conditions and the following disclaimer in the
 //       documentation and/or other materials provided with the distribution.
 //     * Neither the name of Stanford University nor the
 //       names of its contributors may be used to endorse or promote products
 //       derived from this software without specific prior written permission.
 //
 // THIS SOFTWARE IS PROVIDED BY STANFORD UNIVERSITY AND CONTRIBUTORS ``AS IS'' AND ANY
 // EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 // WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 // DISCLAIMED. IN NO EVENT SHALL STANFORD UNIVERSITY AND CONTRIBUTORS BE LIABLE FOR ANY
 // DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 // (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 // LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 // ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 // (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 // SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 //
 //---------------------------------------------------------------------------- 
// Gary Bradski, a pre-vacation doodle July 14, 2005
// Note that this is a YUV pixel model, must have one for each YUV pixel that you care about



/////////////////////////////////////////////////////////////////////////////////////////// 
/* How to call externally

//CONVERT IMAGE TO YUV
cvCvtColor( image, yuvImage, CV_BGR2YCrCb );


  //DECLARATIONS:
#include "yuv_codebook.h"
#define CHANNELS 3   //Could also use just 1 ("Y", brightness)
  //VARIABLES:
codeBook *cB;  //This will be our linear model of the image, a vector of lengh = height*width
int maxMod[CHANNELS]; //Add these (possibly negative) number onto max level when code_element determining if new pixel is foreground
int minMod[CHANNELS]; //Subract these (possible negative) number from min level code_element when determining if pixel is foreground
unsigned cbBounds[CHANNELS]; //Code Book bounds for learning
int nChannels = CHANNELS;
...
//ALLOCATE IT WHEN YOU KNOW THE IMAGE SIZE
imageLen = image->width*image->height;
cB = new codeBook [imageLen];
for(int f = 0; f<imageLen; f++)
{
   cB[f].numEntries = 0;
}
...
//LEARNING BACKGROUND
uchar *pColor; //YUV pointer

if(learn)
{
	pColor = (uchar *)((yuv)->imageData);
	for(c=0; c<imageLen; c++)
	{
		cvupdateCodeBook(pColor, cB[c], cbBounds, nChannels);
		pColor += 3;
	}
	learnCnt += 1;
}


//ELIMINATE SPURIOUS CODEBOOK ENTRIES (FOR SPEED)

int cleanedCnt; //will hold number of codebook entries eliminated
cleanedCnt = 0;
for(c=0; c<imageLen; c++)
{
	cleanedCnt += cvclearStaleEntries(cB[c]);
}
...


//BACKGROUND SEGMENTATION
uchar *pMask,*pColor;
//For connected components bounding box and center of mass if wanted, else can leave out by default
int num = 5; //Just chose 5 arbitrarily, could be 1, 20, anything
CvRect bbs[5];
CvPoint centers[5];

if(modelExists)
{
	pColor = (uchar *)((yuv)->imageData); //3 channel yuv image
	pMask = (uchar *)((mask)->imageData); //1 channel image
	for(c=0; c<imageLen; c++)
	{
		maskQ = cvbackgroundDiff(pColor, cB[c], nChannels, minMod, maxMod);
		*pMask++ = maskQ;
		pColor += 3;
	}
	//This part just to visualize bounding boxes and centers if desired
	cvCopy(mask,maskCC);
	num = 5; //
	cvconnectedComponents(maskCC,1,4.0, &num, bbs, centers);
	for(int f=0; f<num; f++)
	{
		CvPoint pt1, pt2; //Draw the bounding box in white
		pt1.x = bbs[f].x;
		pt1.y = bbs[f].y;
		pt2.x = bbs[f].x+bbs[f].width;
		pt2.y = bbs[f].y+bbs[f].height;
		cvRectangle(maskCC,pt1,pt2, CV_RGB(255,255,255),2);
		pt1.x = centers[f].x - 3; //Draw the center of mass in black
		pt1.y = centers[f].y - 3;
		pt2.x = centers[f].x +3;
		pt2.y = centers[f].y + 3;
		cvRectangle(maskCC,pt1,pt2, CV_RGB(0,0,0),2);
	}
	mw.paint(maskCC,0,1,0);
}

...
//EXAMPEL OF HOW TO ADJUST BACKGROUDN THRESHOLDS
ch[0] = 0; //ch[0]=>y, ch[1]=>u, ch[2]=>v
ch[1] = 1;
ch[2] = 1;
. . . 
case 'u': //modify max classification bounds
	for(n=0; n<nChannels; n++)
		if(ch[n])
			maxMod[n] += 1;
	break;
case 'i': //modify max classification bounds
	for(n=0; n<nChannels; n++)
		if(ch[n])
			maxMod[n] -= 1;
	break;
case '<': //modify min classification bounds (min bound goes lower)
	for(n=0; n<nChannels; n++)
		if(ch[n])
			minMod[n] += 1;
	break;
case '>': //modify min classification bounds (min bound goes higher)
	for(n=0; n<nChannels; n++)
		if(ch[n])
			minMod[n] -= 1;
...

*/
///////////////////////////////////////////////////////////////////////////////////////////////


// Accumulate average and ~std deviation
#ifndef CVYUV_CB
#define CVYUV_CB


#include <cv.h>				// define all of the opencv classes etc.
#include <highgui.h>
#include <cxcore.h>

#define CHANNELS 3

typedef struct ce {
	uchar learnHigh[CHANNELS]; //High side threshold for learning
	uchar learnLow[CHANNELS];  //Low side threshold for learning
	uchar max[CHANNELS];	//High side of box boundary
	uchar min[CHANNELS];	//Low side of box boundary
	int t_last_update;		//This is book keeping to allow us to kill stale entries
	int stale;				//max negative run (biggest period of inactivity)
} code_element;

typedef struct code_book {
	code_element **cb;
	int numEntries;
	int t;					//count every access
} codeBook;

///////////////////////////////////////////////////////////////////////////////////
// int updateCodeBook(uchar *p, codeBook &c, unsigned cbBounds)
// Updates the codebook entry with a new data point
//
// p			Pointer to a YUV pixel
// c			Codebook for this pixel
// cbBounds		Learning bounds for codebook (Rule of thumb: 10)
// numChannels	Number of color channels we're learning
//
// NOTES:
//		cvBounds must be of size cvBounds[numChannels]
//
// RETURN
//	codebook index
int cvupdateCodeBook(uchar *p, codeBook &c, unsigned *cbBounds, int numChannels = 3);

///////////////////////////////////////////////////////////////////////////////////
// uchar cvbackgroundDiff(uchar *p, codeBook &c, int minMod, int maxMod)
// Given a pixel and a code book, determine if the pixel is covered by the codebook
//
// p		pixel pointer (YUV interleaved)
// c		codebook reference
// numChannels  Number of channels we are testing
// maxMod	Add this (possibly negative) number onto max level when code_element determining if new pixel is foreground
// minMod	Subract this (possible negative) number from min level code_element when determining if pixel is foreground
//
// NOTES: 
// minMod and maxMod must have length numChannels, e.g. 3 channels => minMod[3], maxMod[3].
//
// Return
// 0 => background, 255 => foreground
uchar cvbackgroundDiff(uchar *p, codeBook &c, int numChannels, int *minMod, int *maxMod);



//UTILITES////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////
//int clearStaleEntries(codeBook &c)
// After you've learned for some period of time, periodically call this to clear out stale codebook entries
//
//c		Codebook to clean up
//
// Return
// number of entries cleared
int cvclearStaleEntries(codeBook &c);

/////////////////////////////////////////////////////////////////////////////////
//int countSegmentation(codeBook *c, IplImage *I)
//
//Count how many pixels are detected as foreground
// c	Codebook
// I	Image (yuv, 24 bits)
// numChannels  Number of channels we are testing
// maxMod	Add this (possibly negative) number onto max level when code_element determining if new pixel is foreground
// minMod	Subract this (possible negative) number from min level code_element when determining if pixel is foreground
//
// NOTES: 
// minMod and maxMod must have length numChannels, e.g. 3 channels => minMod[3], maxMod[3].
//
//Return
// Count of fg pixels
//
int cvcountSegmentation(codeBook *c, IplImage *I, int numChannels, int *minMod, int *maxMod);

///////////////////////////////////////////////////////////////////////////////////////////
//void cvconnectedComponents(IplImage *mask, int poly1_hull0, float perimScale, int *num, CvRect *bbs, CvPoint *centers)
// This cleans up the forground segmentation mask derived from calls to cvbackgroundDiff
//
// mask			Is a grayscale (8 bit depth) "raw" mask image which will be cleaned up
//
// OPTIONAL PARAMETERS:
// poly1_hull0	If set, approximate connected component by (DEFAULT) polygon, or else convex hull (0)
// perimScale 	Len = image (width+height)/perimScale.  If contour len < this, delete that contour (DEFAULT: 4)
// num			Maximum number of rectangles and/or centers to return, on return, will contain number filled (DEFAULT: NULL)
// bbs			Pointer to bounding box rectangle vector of length num.  (DEFAULT SETTING: NULL)
// centers		Pointer to contour centers vectore of length num (DEFULT: NULL)
// clean		If set (default), clearn image by image morphology
//
void cvconnectedComponents(IplImage *mask, int poly1_hull0, float perimScale, int *num, CvRect *bbs, CvPoint *centers, int clean = 1);


#endif
