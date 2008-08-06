//-------------------------------------------------------------------------------
//
// CaptureObjects.cpp : Defines the entry point for the console application.
//
//    OpenCV based code generating a multi-panel image window.
//    Copyright (C) 2006 Gary Bradski
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
//-------------------------------------------------------------------------------
// 
// This program is an aid to developing a database of captured objects.  It uses simple 
// background subtraction to learn a scene; segments new objects put into the scene and
// when you hit the space bar it writes the raw image, the segmented mask and the bounding 
// box mask out to disk.  Using this, you may rapidly collect a database of objects and their
// ground truth segmentation.  
// 
//


#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cvx_multiwin.h>
#include <stdio.h>
#include <ctype.h>
#include <cv_yuv_codebook.h>


#include "image_utils/cv_bridge.h"
#include <std_msgs/ImageArray.h>
#include <ros/node.h>

extern int CONTOUR_APPROX_LEVEL;   // Approx.threshold - the bigger it is, the simpler is the boundary
extern int CLOSE_ITR;

// Some global variables:
// 
//DEALING WITH BACKGROUND SEGMENTATION PARAMETERS
int maxMod[CHANNELS]; //Add these (possibly negative) number onto max level when code_element determining if new pixel is foreground
int minMod[CHANNELS]; //Subract these (possible negative) number from min level code_element when determining if pixel is foreground
unsigned cbBounds[CHANNELS]; //Code Book bounds for learning
int nChannels = CHANNELS;
int ch[3] = {1,1,1};			//Tells which channels are turned on for threshold modification
//DEALING WITH VIEWING WINDOWS, VIDEO AND ASSOCIATED IMAGES:
enum { SOURCE_AVI, SOURCE_CAM, SOURCE_UNKNOWN } g_video_source = SOURCE_UNKNOWN; 
int g_slider_position = 0;
int g_throttle_position = 1;
CvCapture* capture = 0;
IplImage *image = 0, *markedImage, *mask, *rawMask, *yuv = NULL, *maskCC, *imgBoxedObj;
int paused = 0;			//If = 1, pause the movie
CvPoint pt;
CvxMultiWin mw; //Multi-window 
unsigned long framecount = 0;
int learn = 0, learnTarget = -1, modelExists =0, learnCnt = 0;  //Setup for background learning
int imageLen; 
char ObjectLabel[256],ObjectStore[256],Scratch[300],ext[4];
int ObjectCount = 0;	//Increments each new object stored

//Callback mouse handler
//Flags 0 = no button
// 1 (left)|2(right)|4(middle)
//Mouse Globals:
int lbuttondown = 0, lbuttonrelease = 0;
CvPoint ptNow,ptD,ptU;
codeBook *cB;


// -- ROS Node class for getting Videre images.
class ImgBGSubtr : public ros::node
{
public:
  IplImage *frame;
  std_msgs::ImageArray frame_msg;
  bool hasNewFrameMsg;
  CvBridge<std_msgs::Image> *bridge;

  ImgBGSubtr() : ros::node("img_bg_subtr"), hasNewFrameMsg(false), frame(0)
  {
    bridge = NULL;
    subscribe("videre/images", frame_msg, &ImgBGSubtr::processFrame, true);

  }

  //Copies the image out of the frame_msg and into frame.
  void processFrame() 
  {
//     fprintf(stdout, "Got a videre array.   ");
//     fprintf(stdout, "%d images\n", frame_msg.get_images_size());
    //fprintf(stdout, "%s colorspace\n", frame_msg.images[0]);

    if(!hasNewFrameMsg) {
      if(bridge != NULL) {
	delete bridge;
      }
      //bridge = new CvBridge<std_msgs::Image>(&frame_msg.images[0], CvBridge<std_msgs::Image>::CORRECT_BGR);
      bridge = new CvBridge<std_msgs::Image>(&frame_msg.images[0], CvBridge<std_msgs::Image>::CORRECT_BGR | CvBridge<std_msgs::Image>::MAXDEPTH_8U);
      frame = 0;
      bridge->to_cv(&frame);
      hasNewFrameMsg = true;
    }
    //else
      //      fprintf(stdout, "Deferring..\n");
  }  
};


//JUST A MOUSE EVENT HANDLER, NOT REALLY USED HERE
void on_mouse( int event, int x, int y, int flags,void *foo = NULL )
{
  if( !image )
    return;

  if( image->origin )
    y = image->height - y;  //Mostly this happens.

  ptNow = cvPoint(x,y);
  if( event == CV_EVENT_LBUTTONDOWN )
    {
      ptD = cvPoint(x,y);
      //Set up for write
      lbuttondown = 1;
      printf("\nL button down at (%d, %d)\n",x,y);
    }
  if( event == CV_EVENT_LBUTTONUP)
    {
      lbuttondown = 0;
      lbuttonrelease = 10;
      ptU = cvPoint(x,y);
      printf("L button up at (%d, %d)\n",x,y);
      //unsigned int
      if(ptU.y > ptD.y)
	printf("increase y %d\n",ptU.y-ptD.y);
      else
	printf("decrease y %d\n",ptU.y-ptD.y);

    }
  if( event == CV_EVENT_RBUTTONDOWN ) //Right click just print where you are
    {
      printf("(X,Y)=(%d, %d)\n",x,y);
    }
}

//TRACK BAR HANDLER JUST FOR FUN, NOT REALLY USED HERE
void onTrackbarSlide(int pos) {
  printf("Slider to %d\n", pos );
  cvSetCaptureProperty( capture, CV_CAP_PROP_POS_FRAMES, pos*100 );
}

//BACKGROUND SUBTRACTION SUPPORT
float avgEntries()
{
  float acc=0;
  for(int c=0; c<imageLen; c++)
    {
      acc += (float)cB[c].numEntries;
    }
  return(acc/(float)imageLen);
}

//Write bounding box rectangle to a file	
void writeBB(const char *name, CvRect &r)
{
  FILE *fptr;
  fptr = fopen(name,"w");
  fprintf(fptr,"r.x %d r.y %d r.w %d r.h %d",r.x,r.y,r.width,r.height);
  fclose(fptr);
}

int maxEntries()
{
  int max=0;
  for(int c=0; c<imageLen; c++)
    {
      if(max < cB[c].numEntries)
	max = cB[c].numEntries;
    }
  return(max);
}

//Read in set thresholds
//0 OK, 1 error
int readThresholds()
{
  FILE *fptr = NULL;
  int i;
  char foo[64];
  if (!(fptr = fopen("thresholds.txt","r")))
    {
      maxMod[0] = 3;	//Default color thresholds to more likely values
      minMod[0] = 10;
      maxMod[1] = 1;
      minMod[1] = 1;
      maxMod[2] = 1;
      minMod[2] = 1;
      fptr = fopen("thresholds.txt","w");
      if(!fptr) 
	return -1;
      for(i=0; i<CHANNELS; i++)
	fprintf(fptr, "maxMod%d %d ",i,maxMod[i]);
      for(i=0; i<CHANNELS; i++)
	fprintf(fptr, "minMod%d %d ",i,minMod[i]);
    }
  else
    {
      for(i=0; i<CHANNELS; i++)
	fscanf(fptr,"%s %d ",foo,&maxMod[i]);
      for(i=0; i<CHANNELS; i++)
	fscanf(fptr,"%s %d ",foo,&minMod[i]);
    }
  fclose(fptr);
  return 0;
}
//-1 fail, 0 OK.
int writeThresholds()
{
  FILE *fptr = NULL;
  int i;
  if (!(fptr = fopen("thresholds.txt","w")))
    return -1;
  for(i=0; i<CHANNELS; i++)
    fprintf(fptr, "maxMod%d %d ",i,maxMod[i]);
  for(i=0; i<CHANNELS; i++)
    fprintf(fptr, "minMod%d %d ",i,minMod[i]);
  fclose(fptr);
  return 0;
}


// COMMAND LINE HELP FUNCTION //
void help()
{
  printf( "\n\nCaptureObjects -- Help build a database of objects.\n\n"
	  "WHAT:\n"
	  "CaptureObjects -- Help build a database of objects.\n"
	  "Learn a background of the scene; Place objects in it;\n"
	  "Then hitting \"SPACE\" Store's the raw image, mask, object image and bounding box files named:\n"
	  "  <name>.bmp (image), <name>Mask.bmp (object mask),\n"
	  "  <name>Rect.txt (tight rectangle), <name>BigRect.txt (Rectange buffered by 19 on each side for Poggio)\n"
	  "\n"
	  "HOW IT WORKS:\n"
	  "Forms a code book of boxes in YUV space,\n"
	  "high low thresholds on these boxes may be changed.\n"
	  "New pixels outside of the YUV boxes are declared foreground.\n"
	  "\n"
	  "WATCH OUT FOR!:\n"
	  "Reflections, shadows, white or object colored backgrounds.\n"
	  "Click on the multiwin not consol to get text response in consol\n"
	  "Best to turn off AGC (automatic gain control) and WB (White balance)\n"
	  "\n"
	  "INSTRUCTIONS: \n"
	  "Mouse focus must be on the multiwindow, printf's go to consol window then\n"
	  "Press \"l\" to learn a background for 120 frames.\n"
	  "Adjust thresholds.  Channel 0 is brightness, channel 4 is both colors together. \".\" toggles\n"
	  "Press N,n to give your object a base name,\n" 
	  "SPACE BAR to store an object (make sure there's only one box around it)\n"
	  "Once happy, start capturing objects. Base name of object increments by one each time it's stored.\n"
	  "		\n"
	  "Hot keys: \n"
	  "\tESC,q - quit the program\n"
	  "\th   - print this help menue\n"
	  "\n"
	  "********Learning and Threshold adjustment:********\n"
	  "\tl   - Clear old model and learn new background model for 120 frames\n"
	  "\tL   - Learn on/off toggle\n"
	  "\tC   - Clear background model\n"
	  "\n"
	  "********Threshold adjustment:********\n"
	  "\tz   - Max class bounds ++\n"
	  "\tx   - Max class bounds --\n"
	  "\t[   - Min class bounds ++\n"
	  "\t]   - Min class bounds --\n"
	  "\td   - Restore threshold defaults.\n"
	  "\n"
	  "********YUV Image; Which thresholds to change:********\n"
	  "\t.   - Toggle between channel 0 (Y) and 4 (U,V)\n"
	  "\t0   - Threshold adj channel 0 (Y or brightness) only\n"
	  "\t1   - Threshold adj channel 1 (U Color) only\n"
	  "\t2   - Threshold adj channel 2 (V Color) only\n"
	  "\t3   - Threshold adj all channels\n"
	  "\t4   - Threshold adj channels 2,3 (color channels U,V) only\n"
	  "\t,   - toggle channels 3,1 \n"
	  "\n"
	  "********STORE:********\n"
	  "\tn,N - Name your object!\n"
	  "\tSPACE -- Stores raw image, mask, object, text bounding box\n"
	  "\t           and expanded box\n"
	  "\t         Pressing again increments a number:\n"
	  "\t           obj0.png, obj1.png, obj2.png ...\n"
	  "\n"
	  "********UTILITY:********\n"
	  "\te   - Extension -- choice of image format to store. Default=png\n"
	  "\tR,W - W: Write out code book thresholds, R: Read them in (done on lauch)\n"
	  "\n"
	  "********Misc:********\n"
	  "\tp   - Pause toggle\n"
	  "\tr   - Remove stale codebook entries (can make code faster)\n"
	  "\ta   - Return average codebook entries\n"
	  "Mouse down in image window activated mouse event\n" );
}


// Calls:	CaptureObjects  <path\>filename	//Run movie file
//			CaptureObjects					//Use camera
//			CaptureObjects	#				//Use just this function
int main( int argc, char** argv )
{
  //Main scope globals:
  int c,n;
  uchar *pColor,*pMask,maskQ;
  FILE *fRectOut = NULL;
  int numBoxRegions = 5;
  CvRect bbs[5];
  CvPoint centers[5];
  CvPoint pt1, pt2; //For the above rectangles Draw the bounding box in white
  int train = 0, oldframecount = 0;
  sprintf(ObjectLabel,"DefaultObject");
  sprintf(ext,"png");
  int ii = (int)strlen(ObjectLabel);
//   //From Camera
//   if( argc == 1 || (argc == 2 && strlen(argv[1]) == 1 && isdigit(argv[1][0])))
//     {
//       capture = cvCaptureFromCAM( argc == 2 ? argv[1][0] - '0' : 0 );
//       g_video_source = SOURCE_CAM;
//       fprintf(stdout, "Capturing from cam %d\n", argc == 2 ? argv[1][0] - '0' : 0);
//     }
//   else if( argc == 2 ) //Or from AVI
//     {
//       capture = cvCaptureFromAVI( argv[1] );
//       g_video_source = SOURCE_AVI;
//       fprintf(stdout, "Capturing from avi.\n");
//     }

//   if( !capture )
//     {
//       fprintf(stderr,"Could not initialize capturing...\n");
//       return -1;
//     }

  help();

  //Create Window
  cvNamedWindow( "CaptureObjects", 0 );
  //Create Mouse Handler
  //	CVAPI(void) cvSetMouseCallback( const char* window_name, CvMouseCallback on_mouse,
  //                                void* param CV_DEFAULT(NULL));

  cvSetMouseCallback( "CaptureObjects", on_mouse );
  //Create Trackbars
//   if( g_video_source == SOURCE_AVI ) {
//     int frames = (int) cvGetCaptureProperty( capture, CV_CAP_PROP_FRAME_COUNT );
//     printf("TOTAL FRAMES = %d\n", frames ); 
//     cvCreateTrackbar(
// 		     "Position(k)",
// 		     "CaptureObjects",
// 		     &g_slider_position,
// 		     frames/100,
// 		     onTrackbarSlide
// 		     );
//   }
//   int frames = (int) cvGetCaptureProperty( capture, CV_CAP_PROP_FRAME_COUNT );
//   printf("TOTAL FRAMES = %d\n", frames );
//   cvCreateTrackbar(
// 		   "Throttle",
// 		   "CaptureObjects",
// 		   &g_throttle_position,
// 		   1,
// 		   NULL
// 		   );
 

  // -- Set up for getting images from ROS.
  ros::init(argc, argv);
  ImgBGSubtr node;
  while(!node.hasNewFrameMsg) {
    usleep(1000);
  }
  IplImage* frame = cvCreateImage(cvGetSize(node.frame), 8, 3);



  /////////////////FRAME PROCESS LOOP////////////////////////////
  for(;;)
    {
      // -- Check for control-c.
      if(!node.ok())
	break;

      if(!paused && node.hasNewFrameMsg)
	{
	 
	  // -- Get frame from ROS rather than from the camera directly.
	  node.hasNewFrameMsg = false;
	  cvMerge( node.frame, node.frame, node.frame, NULL, frame ); 

	  // -- Continue with normal execution of CaptureObjects.
	  framecount += 1;
	  if(!(framecount%20))
	    {
	      float afoo = 0.0;
	      int max = 0;
	      if(framecount > 10)
		{afoo = avgEntries(); max = maxEntries();}
	      printf("frame=%d, avgEntries=%f, maxEntries=%d\n",framecount,afoo,max);
	    }
	  if( !frame )
	    break;

	  if( !image ) 
	    {
	      /* allocate all the buffers */
	      image = cvCreateImage( cvGetSize(frame), 8, 3 );
	      image->origin = frame->origin;
	      markedImage = cvCloneImage(image);

	      int myWidth = image->width;
	      int myHeight = image->height;
	      int nChan = image->nChannels;
	      printf("Width=%d, Height=%d, Channels=%d, origin=%d\n",myWidth,myHeight,nChan,image->origin);
	      mw.initialize("Segmentation", image->width, image->height, 2,2,8,3,0,image->origin); 
	      mw.set_label( 0, 0, "c1 r1 CONNECTED COMPONENTS" );
	      mw.set_label(1,0,"c2 r1 BOUNDING BOX");
	      mw.set_label(0,1,"c1 r2 RAW IMAGE");
	      mw.set_label(1,1,"c2 r2 RAW SEGMENTATION");
	      mask = cvCreateImage( cvGetSize(frame), 8, 1 );
	      mask->origin = image->origin;
	      rawMask = cvCreateImage( cvGetSize(frame), 8, 1 );
	      rawMask->origin = image->origin;	
	      maskCC = cvCreateImage( cvGetSize(frame), 8, 1 ); //connected components mask
	      maskCC->origin = image->origin;
	      //				mask = cvCloneImage( image );
	      if( yuv==NULL ) yuv = cvCreateImage( cvGetSize(image), IPL_DEPTH_8U, 3) ;
	      yuv->origin = 1;
	      //				yuvRGB = cvCloneImage(yuv);

	      cvZero( yuv );
	      //randI = cvCloneImage(yuv);
	      imageLen = image->width*image->height;
	      cB = new codeBook [imageLen];
	      for(int f = 0; f<imageLen; f++)
		{
		  cB[f].numEntries = 0;
		}
	      for(n=0; n<nChannels;n++)
		{
		  cbBounds[n] = 10; //Learning bounds factor
		}
	      readThresholds(); //Read in default codebook thresholds
	    }

// 	  fprintf(stdout, "Copying...\n");
// 	  fprintf(stdout, "frame nchannels: %d  depth: %d\n", frame->nChannels, frame->depth);
// 	  fprintf(stdout, "image nchannels: %d  depth: %d\n", image->nChannels, image->depth);
	  cvCopy(frame, image, 0 );
// 	  fprintf(stdout, "Done\n");
	  cvCopy(frame, markedImage); //Transfer image to get marked up
	  cvCvtColor( image, yuv, CV_BGR2YCrCb ); //Work in a brightness factorized space

	  //BACKGROUND LEARNING
	  if(learn)
	    {
	      pColor = (uchar *)((yuv)->imageData);
	      for(c=0; c<imageLen; c++)
		{
		  cvupdateCodeBook(pColor, cB[c], cbBounds, nChannels);
		  pColor += 3;
		}
	      learnCnt += 1;
	      if(learnCnt == learnTarget) //If doing a set amount of learning
		{
		  learnTarget = -1;
		  learn = 0;
		  modelExists = 1;
		  printf("Learning Complete\n");
		}
	    }

	  //MOUSE HANDLER
	  if(lbuttonrelease)
	    {
	      lbuttonrelease -=1;
	    }
	} //end if not paused

      //BACKGROUND SEGMENTATION
      if(modelExists)
	{
	  pColor = (uchar *)((yuv)->imageData);
	  pMask = (uchar *)((mask)->imageData);
	  //DO RAW SEGMENTATION
	  for(c=0; c<imageLen; c++)
	    {
	      maskQ = cvbackgroundDiff(pColor, cB[c], nChannels, minMod, maxMod);
	      *pMask++ = maskQ;
	      pColor += 3;
	    }
	  mw.paint(mask,1,1,0); //Raw Segmented Image
	  cvCopy(mask,rawMask); //Save raw mask pixels

	  cvCopy(mask,maskCC); //For connected components
	  numBoxRegions = 5;
	  cvconnectedComponents(maskCC,1,4.0, &numBoxRegions, bbs, centers);
	  cvCopy(maskCC,mask); //Keep a clean copy of the connected components
	  mw.paint(maskCC,0,0,0); //CONNECTED COMPONENTS
	  for(int f=0; f<numBoxRegions; f++)
	    {
	      pt1.x = bbs[f].x;
	      pt1.y = bbs[f].y;
	      pt2.x = bbs[f].x+bbs[f].width;
	      pt2.y = bbs[f].y+bbs[f].height;
	      cvRectangle(maskCC,pt1,pt2, CV_RGB(255,255,255),2);
	      cvRectangle(markedImage,pt1,pt2,CV_RGB(255,155,55),2);
	      pt1.x = centers[f].x - 3; //Draw the center of mass in black
	      pt1.y = centers[f].y - 3;
	      pt2.x = centers[f].x +3;
	      pt2.y = centers[f].y + 3; 
	      cvRectangle(maskCC,pt1,pt2, CV_RGB(0,0,0),2);
	    }
	  mw.paint(maskCC,1,0,0); //Segment with bounding box
	}

      cvShowImage( "CaptureObjects", image );
      mw.paint(markedImage,0,1,1);  //Raw image with marked bounding boxes

      //Get keyboard input
      c = cvWaitKey(10);
      if( (c == 27)||(c == 'q') )
	break;
      int bgCnt,pbgCnt, origbgCnt;
      char bg[10];
      int jj;
      switch( c )
        {
	case ' ': //Write out object
	  if(!modelExists)
	    {
	      printf("Can't write image, no background was learned, press \"l\" first.\n");
	      break;
	    }
	  if(!numBoxRegions) //Must have found an object
	    {
	      printf("Can't write image, no boxed object has been found\n");
	      break;
	    }
	  printf("Writing out %s%d[.%s,Mask.%s,Rect.txt,BigRect.txt] Big if 20 wider on each side\n",ObjectLabel,ObjectCount,ext,ext);
	  //Out Goes the raw iamge
	  sprintf(Scratch,"%sRaw%.3d.%s",ObjectLabel,ObjectCount,ext);
	  cvSaveImage(Scratch,image);

	  //Out goes the connected component mask image xxx
	  sprintf(Scratch,"%sMask%.3d.%s",ObjectLabel,ObjectCount,ext);
	  cvMorphologyEx( rawMask, rawMask, NULL, NULL, CV_MOP_OPEN, 1 );
	  cvMorphologyEx( rawMask, rawMask, NULL, NULL, CV_MOP_CLOSE, 1 );
	  cvAnd(mask,rawMask,mask,mask);
	  cvSaveImage(Scratch,mask);

	  //Out goes the clipped out object (first bounding box)
	  sprintf(Scratch,"%sObj%.3d.%s",ObjectLabel,ObjectCount,ext);
	  imgBoxedObj = cvCreateImage(cvSize(bbs[0].width,bbs[0].height),8,3);
	  imgBoxedObj->origin = image->origin; //Make sure clipped image is not upside down
	  cvSetImageROI(image,cvRect( bbs[0].x, bbs[0].y, bbs[0].width, bbs[0].height));
	  cvCopy(image,imgBoxedObj); //Copy ROI from image to boxed image
	  cvSaveImage(Scratch,imgBoxedObj); //Save the object patch
	  //Clean up
	  cvResetImageROI(image);  //Release the ROI
	  cvReleaseImage( &imgBoxedObj ); //Release the temporary boxed object image

	  //Out Goes the first rectangle -- hope you hit space when only rectangle was there
	  sprintf(Scratch,"%s%.3dRect.txt",ObjectLabel,ObjectCount);
	  fRectOut = fopen(Scratch,"w");
	  if(!fRectOut)
	    printf("ERROR, couldn't write %s\n",Scratch);
	  else
	    {
	      for(int br=0; br<numBoxRegions; br++)
		{
		  pt1.x = bbs[br].x;
		  pt1.y = bbs[br].y;
		  pt2.x = bbs[br].x+bbs[br].width;
		  pt2.y = bbs[br].y+bbs[br].height;
		  fprintf(fRectOut,"pt1.x %d pt1.y %d pt2.x %d pt2.y %d\n",pt1.x,pt1.y,pt2.x,pt2.y);
		}
	      fclose(fRectOut);
	    }
	  //Out Goes the rectangle with 20 pixel buffer (to allow room for Poggio feature buffering)
	  sprintf(Scratch,"%s%.3dBigRect.txt",ObjectLabel,ObjectCount);
	  fRectOut = fopen(Scratch,"w");
	  if(!fRectOut)
	    printf("ERROR, couldn't write %s\n",Scratch);
	  else
	    {
	      for(int br2=0; br2<numBoxRegions; br2++)
		{
		  pt1.x = bbs[br2].x;
		  pt1.y = bbs[br2].y;
		  pt2.x = bbs[br2].x+bbs[br2].width;
		  pt2.y = bbs[br2].y+bbs[br2].height;
		  pt1.x -= 19;
		  if(pt1.x < 0) pt1.x = 0;
		  pt1.y -=19;
		  if(pt1.y < 0) pt1.y = 0;
		  pt2.x += 19;
		  if(pt2.x >= image->width) pt2.x = image->width - 1;
		  pt2.y += 19;
		  if(pt2.y >= image->height) pt2.y = image->height - 1;
		  fprintf(fRectOut,"pt1.x %d pt1.y %d pt2.x %d pt2.y %d\n",pt1.x,pt1.y,pt2.x,pt2.y);
		}
	      fclose(fRectOut);
	    }
	  ObjectCount += 1;
	  break;
	case 'R': //Read in thresholds
	  printf("Reading codebook thresholds\n");
	  if(readThresholds())
	    printf("Couldn't read in codebook max and min thresholds, defaulting...\n");
	  break;
	case 'W':
	  printf("Writing codebook thresholds\n");
	  if(writeThresholds())
	    printf("Failed to write out thresholds\n");
	  break;
	case 'n'://Label your object
	case 'N':
	  printf("\nObject path\\name: (don't use extensions or funny characters)\nESC to cancel\n");
	  printf("%s\r",ObjectLabel);
	  strcpy(ObjectStore,ObjectLabel);
	  while((c != 27)&&(c != 13)&&(ii < 254))
	    {
	      c = cvWaitKey();
	      if(c == 13) break;
	      if(c == ' ') continue;
	      if(c == '\t') continue;
	      if(c == 8)
		{
		  printf("\r");
		  for(jj = 0; jj<=ii; jj++)
		    printf(" ");
		  printf("\r");
		  ii -= 1;
		  if (ii <= 0) 
		    ii = 0;
		  ObjectLabel[ii] = 0;
		}
	      else
		{
		  ObjectLabel[ii] = c;
		  ii++;
		  ObjectLabel[ii] = 0;
		}
	      printf("%s\r",ObjectLabel);
	    } 
	  if(c == 27) //ESC => restore old label state
	    strcpy(ObjectLabel,ObjectStore);
	  else
	    ObjectCount = 0;
	  printf("\nFinal label:\n%s\n",ObjectLabel);
	  break;
	case 'e': //Choose image storage extension file type.
	  printf("\nWhich file type do you want to store? 0 = png, 1 = bmp, 2 = jpg, 3=pbm, 4=TIFF\n");
	  c = cvWaitKey();
	  switch(c)
	    {
	    case '0': //png
	      strcpy(ext,"png");
	      break;
	    case '1': // bmp
	      strcpy(ext,"bmp");
	      break;
	    case '2': // jpg
	      strcpy(ext,"jpg");
	      break;
	    case '3': // jpg
	      strcpy(ext,"pbm");
	      break;
	    case '4': // jpg
	      strcpy(ext,"tif");
	      break;
	    default:
	      strcpy(ext,"png");
	      break;
	    }
	  printf("\nExtension is .%s\n",ext);
	  break;
	case 'C': //Clear codebook
	  printf("\nDeleating codebook!\n");
	  delete [] cB;
	  printf("Reallocate new ... \n");
	  cB = new codeBook [imageLen];
	  for(int f = 0; f<imageLen; f++)
	    {
	      cB[f].numEntries = 0;
	    }
	  for(n=0; n<nChannels;n++)
	    {
	      cbBounds[n] = 10; //Learning bounds factor
	    }
	  modelExists = 0;  //No model now exists
	  learnTarget = -1; //Turn off any learn counts that exist
	  learn = 0;        //Not learning
	  printf("you need to learn\n");
	  break;
	case 'l': //Learn for 120 frames
	  if(modelExists)
	    { //Kill old codebook
	      printf("\nKilling old codebook!\n");
	      delete [] cB;
	      printf("Reallocate new ... \n");
	      cB = new codeBook [imageLen];
	      for(int f = 0; f<imageLen; f++)
		{
		  cB[f].numEntries = 0;
		}
	      for(n=0; n<nChannels;n++)
		{
		  cbBounds[n] = 10; //Learning bounds factor
		}
	      modelExists = 0;  //No model now exists
	    }
	  printf("Learning on for 120 frames, use \"L\" if you want to manually toggle on and off...\n");
	  learn = 1;
	  learnTarget = learnCnt + 120;
	  break;
	case 'L': //Toggle learning
	  if(learn) 
	    {
	      learn = 0; 
	      modelExists = 1;
	      pbgCnt = cvcountSegmentation(cB, yuv, nChannels, minMod, maxMod);
	      origbgCnt = pbgCnt+1;
	      printf("minMod = ");
	      for(n=0; n<nChannels; n++)
		printf("%d, ",minMod[n]);
	      printf("; maxMod = ");
	      for(n=0; n<nChannels; n++)
		printf("%d, ",maxMod[n]);
	      printf("; ");
	      printf("bg=%d \n",pbgCnt);
	    }
	  else learn = 1;
	  printf("learn=%d\n",learn);
	  break;
	case ',': //Toggle looking at Y or color
	  if(modelExists && (CHANNELS == 3))
	    {
	      if(nChannels == 3)
		nChannels = 1;
	      else
		nChannels = 3;
	    }
	  printf("nChannels = %d\n",nChannels);
	  break;
	case 'r': //Remove stale codebook entries
	  {
	    int cleanedCnt;
	    cleanedCnt = 0;
	    for(c=0; c<imageLen; c++)
	      {
		cleanedCnt += cvclearStaleEntries(cB[c]);
	      }
	    printf("Number of codebook entried deleted = %d;\n",cleanedCnt);
	  }
	  break;
	case 'a': //return average number of codebook entries
	  float avg;
	  avg = avgEntries();
	  printf("Average number of codebook entries is %f\n",avg);
	  break;
	case '.': //toggle between setting Y and UV
	  if(ch[0])
	    {
	      ch[0] = 0;
	      ch[1] = 1;
	      ch[2] = 1;
	      printf("Adjust color UV only\n");
	    }
	  else
	    {
	      ch[0] = 1;
	      ch[1] = 0;
	      ch[2] = 0;
	      printf("Adjust brightness Y only\n");
	    }
	  break;
	case '0':
	  ch[0] = 1;
	  ch[1] = 0;
	  ch[2] = 0;
	  printf("Adjust brightness Y only Channels active: ");
	  for(n=0; n<nChannels; n++)
	    printf("%d, ",ch[n]);
	  printf("\n");
	  break;
	case '1':
	  ch[0] = 0;
	  ch[1] = 1;
	  ch[2] = 0;
	  printf("Adjust color U only Channels active: ");
	  for(n=0; n<nChannels; n++)
	    printf("%d, ",ch[n]);
	  printf("\n");
	  break;
	case '2':
	  ch[0] = 0;
	  ch[1] = 0;
	  ch[2] = 1;
	  printf("Adjust color V only Channels active: ");
	  for(n=0; n<nChannels; n++)
	    printf("%d, ",ch[n]);
	  printf("\n");
	  break;
	case '3':
	  ch[0] = 1;
	  ch[1] = 1;
	  ch[2] = 1;
	  printf("Adjust all Channels active: ");
	  for(n=0; n<nChannels; n++)
	    printf("%d, ",ch[n]);
	  printf("\n");
	  break;
	case '4':
	  ch[0] = 0;
	  ch[1] = 1;
	  ch[2] = 1;
	  printf("Adjust colors UV only Channels active: ");
	  for(n=0; n<nChannels; n++)
	    printf("%d, ",ch[n]);
	  printf("\n");
	  break;
	case 'z': //modify max classification bounds
	  for(n=0; n<nChannels; n++)
	    if(ch[n])
	      maxMod[n] += 1;
	  bgCnt = cvcountSegmentation(cB, yuv, nChannels, minMod, maxMod);
	  if(bgCnt > pbgCnt) sprintf(bg,"^");
	  else sprintf(bg,"v");
	  pbgCnt = bgCnt;
	  printf("maxMod = ");
	  for(n=0; n<nChannels;n++)
	    {
	      printf("%d, ",maxMod[n]);
	    }
	  printf("; ");
	  printf("bg=%s(%4.2f%%)%d \n",bg,100.0*(bgCnt - origbgCnt)/origbgCnt, bgCnt);
	  break;
	case 'x': //modify max classification bounds
	  fprintf(stdout, "Got a }\n");
	  for(n=0; n<nChannels; n++)
	    if(ch[n])
	      maxMod[n] -= 1;
	  bgCnt = cvcountSegmentation(cB, yuv, nChannels, minMod, maxMod);
	  if(bgCnt > pbgCnt) sprintf(bg,"^");
	  else sprintf(bg,"v");
	  pbgCnt = bgCnt;
	  printf("maxMod = ");
	  for(n=0; n<nChannels;n++)
	    {
	      printf("%d, ",maxMod[n]);
	    }
	  printf("; ");
	  printf("bg=%s(%4.2f%%)%d \n",bg,100.0*(bgCnt - origbgCnt)/origbgCnt, bgCnt);
	  break;
	case '[': //modify min classification bounds (min bound goes lower)
	  for(n=0; n<nChannels; n++)
	    if(ch[n])
	      minMod[n] += 1;
	  bgCnt = cvcountSegmentation(cB, yuv, nChannels, minMod, maxMod);
	  if(bgCnt > pbgCnt) sprintf(bg,"^");
	  else sprintf(bg,"v");
	  pbgCnt = bgCnt;
	  printf("minMod = ");
	  for(n=0; n<nChannels;n++)
	    {
	      printf("%d, ",minMod[n]);
	    }
	  printf("; ");
	  printf("bg=%s(%4.2f%%)%d\n",bg,100.0*(bgCnt - origbgCnt)/origbgCnt, bgCnt);
	  break;
	case ']': //modify min classification bounds (min bound goes higher)
	  for(n=0; n<nChannels; n++)
	    if(ch[n])
	      minMod[n] -= 1;
	  bgCnt = cvcountSegmentation(cB, yuv, nChannels, minMod, maxMod);
	  if(bgCnt > pbgCnt) sprintf(bg,"^");
	  else sprintf(bg,"v");
	  pbgCnt = bgCnt;
	  printf("minMod = ");
	  for(n=0; n<nChannels;n++)
	    {
	      printf("%d, ",minMod[n]);
	    }
	  printf("; ");
	  printf("bg=%s(%4.2f%%)%d\n",bg,100.0*(bgCnt - origbgCnt)/origbgCnt, bgCnt);
	  break;
	case 'd': //Restore threshold defaults
	  maxMod[0] = 3;	//Default color thresholds to more likely values
	  minMod[0] = 10;
	  maxMod[1] = 1;
	  minMod[1] = 1;
	  maxMod[2] = 1;
	  minMod[2] = 1;
	  break;
	case 'p': //Pause
	  if(paused == 1)
	    {
	      paused = 0;
	      printf("Unpaused\n");
	    }
	  else
	    {
	      paused = 1;
	      printf("Paused\n");
	    }
	  break;
	case 'h':
	  help();
	  break;
	default:
	  ;
        }
    }
  //CLEAN UP
  //cvReleaseCapture( &capture );
  cvDestroyWindow("CaptureObjects");
  cvReleaseImage( &image );
  cvReleaseImage( &markedImage );
  cvReleaseImage( &mask);
  cvReleaseImage( &maskCC);
  cvReleaseImage( &yuv);
  delete [] cB;
  /*	for(int p=0; p<ptWriteCnt; p++)
	{
	fclose(pPt[p]);
	}
  */
  
  ros::fini();
  
  return 0;
}

