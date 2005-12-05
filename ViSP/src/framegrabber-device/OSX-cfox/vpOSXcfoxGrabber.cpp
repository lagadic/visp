/*                                                                -*-c++-*-
#----------------------------------------------------------------------------
#  Copyright (C) 2005  IRISA-INRIA Rennes Vista Project
#  All Rights Reserved.
#
#
#    Contact:
#       Fabien Spindler
#       IRISA-INRIA Rennes
#       Campus Universitaire de Beaulieu
#       35042 Rennes Cedex
#       France
#
#    email: fspindle@irisa.fr
#    www  : http://www.irisa.fr/lagadic
#
#----------------------------------------------------------------------------
*/

/*!
  \file vpV4l2Grabber.cpp
  \brief class for the Video For Linux 2 video device
  \ingroup libdevice
*/

#include <visp/vpConfig.h>

#ifdef HAVE_LIBCFOX

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/mman.h>
#include <errno.h>
#include <iostream>

#include <visp/vpOSXcfoxGrabber.h>
#include <visp/vpFrameGrabberException.h>
#include <visp/vpImageIo.h>

/*!

Convert YUV422 into RGBa
yuv422 : u y1 v y2 u y3 v y4

*/
#define YUV2RGB(y, u, v, r, g, b)\
  r = 0.9999695*y - 0.0009508*(u-128) + 1.1359061*(v-128);\
  g = 0.9999695*y - 0.3959609*(u-128) - 0.5782955*(v-128);\
  b = 0.9999695*y + 2.04112*(u-128) - 0.0016314*(v-128);\
  r = r < 0 ? 0 : r;\
  g = g < 0 ? 0 : g;\
  b = b < 0 ? 0 : b;\
  r = r > 255 ? 255 : r;\
  g = g > 255 ? 255 : g;\
  b = b > 255 ? 255 : b



static void ConvertYUV422ToRGBa(unsigned char* yuv, unsigned char* rgba,
			  int numpixels)
{

 int i=0,j=0;
 double r, g, b;

 while( j < numpixels*2)
 {

   YUV2RGB (yuv[j+1], yuv[j], yuv[j+2], r, g, b);
   rgba[i] =  (unsigned char) r;
   rgba[i+1] = (unsigned char) g;
   rgba[i+2] = (unsigned char)b;
   rgba[i+3] = 0;
   i+=4;

   YUV2RGB (yuv[j+3], yuv[j], yuv[j+2], r, g, b);
   rgba[i] = (unsigned char)r;
   rgba[i+1] = (unsigned char)g;
   rgba[i+2] =  (unsigned char)b;
   rgba[i+3] = 0;
   i+=4;
   j+=4;

 }

}

static void ConvertYUV422toUchar(unsigned char* yuv, unsigned char* img,
			  int numpixels)
{

 int i=0,j=0;

 while( j < numpixels*2)
 {


   img[i++] = yuv[j+1] ;
   img[i++] = yuv[j+3] ;

   j+=4;

 }

}


const int vpOSXcfoxGrabber::DEFAULT_INPUT = 0;
const int vpOSXcfoxGrabber::DEFAULT_SCALE = 2;

/*!
  \brief constructor


*/
vpOSXcfoxGrabber::vpOSXcfoxGrabber()
{

  setFramerate(vpOSXcfoxGrabber::framerate_30fps);
  setInput(vpOSXcfoxGrabber::DEFAULT_INPUT);
  setScale(vpOSXcfoxGrabber::DEFAULT_SCALE);

  init = false;
}


/*!
  Destructor.

  \sa close() ;
*/
vpOSXcfoxGrabber::~vpOSXcfoxGrabber()
{
  close() ;
}

/*!
  Set the video port
*/
void
vpOSXcfoxGrabber::setInput(unsigned input)
{
  this->input = input;
}

/*!
  Set the decimation factor.

  \exception settingError : Wrong scale (shoud be between 1 and 16).

  \param scale : Decimation factor.
*/
void
vpOSXcfoxGrabber::setScale(unsigned scale)
{
  if ((scale <1) || (scale >2))
  {
    close();

    ERROR_TRACE("Wrong scale %d, scale shoud be between 1 and 2",scale) ;
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "Wrong scale") );
  }

  this->scale = scale ;
}

/*!
  Initialize image acquisition.

  \param I : Image data structure (8 bits image)

  \exception settingError : Wrong input channel.

*/
void
vpOSXcfoxGrabber::open(vpImage<unsigned char> &I)
{
  open();

  I.resize(nrows, ncols) ;

}

/*!
  Initialize image acquisition

  \param I : Image data structure (32 bits image)

  \exception settingError : Wrong input channel.
*/
void
vpOSXcfoxGrabber::open(vpImage<vpRGBa> &I)
{
  open();

  I.resize(nrows, ncols) ;

}



/*!
  Acquire a grey level image.

  \param I : Image data structure (8 bits image)

  \exception initializationError : Frame grabber not initialized.

  \sa getField()
*/
void
vpOSXcfoxGrabber::acquire(vpImage<unsigned char> &I)
{
 if (init==false)
  {
    close();

    throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				   "V4l2 frame grabber not initialized") );
  }

  else
    {
      CFRunLoopRunInMode(kCFRunLoopDefaultMode,1, true);
       Camera::Frame  img = cam.getFrame(MODE_FRAME,0,0) ;
      ConvertYUV422toUchar(img,(unsigned char *)I.bitmap,I.getRows()*I.getCols()) ;

    }

}


/*!
  Acquire a color image.

  \param I : Image data structure (32 bits image)

  \exception initializationError : Frame grabber not initialized.

  \sa getField()
*/
void
vpOSXcfoxGrabber::acquire(vpImage<vpRGBa> &I)
{

  if (init==false)
  {
    close();

    throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				   "V4l2 frame grabber not initialized") );
  }

  else
    {
      CFRunLoopRunInMode(kCFRunLoopDefaultMode,1, true);
      Camera::Frame  img = cam.getFrame(MODE_FRAME,0,0) ;
      ConvertYUV422ToRGBa(img,(unsigned char *)I.bitmap,I.getRows()*I.getCols()) ;

    }

}

/*!

  Set the framerate of the acquisition.

  \param framerate The framerate for the acquisition.

  \sa getFramerate()

*/
void
vpOSXcfoxGrabber::setFramerate(vpOSXcfoxGrabber::framerateEnum _framerate)
{
  framerate = _framerate ;
}

/*!

  Return the framerate of the acquisition.

  \return The actual framerate of the framegrabber.

  \sa setFramerate()
*/


vpOSXcfoxGrabber::framerateEnum
vpOSXcfoxGrabber::getFramerate()
{
  return framerate;
}


/*!
  Close the video device.
*/
void
vpOSXcfoxGrabber::close()
{
  if (init==true)
    cam.stop() ;
}

/*!

  Open the Video For Linux Two device.

  \exception initializationError : Can't access to video device.
  \exception otherError : Can't query video capabilities.

*/
void
vpOSXcfoxGrabber::open()
{

  if (init==true)
    close() ;


  Spec sp = BasicCamera ;
  Mode mode ;
  FPS fps ;

  if (framerate == framerate_30fps)
    fps = FPS_30 ;
  else
    fps = FPS_15 ;

  if  (scale == 2)
  {
      nrows = 240 ; ncols = 320 ;
    mode =       Mode_320x240_YUV422 ;
  }
  else
    //    if (scale ==1)
    {
      mode =       Mode_640x480_YUV422 ;
      nrows = 480 ; ncols = 640 ;
    }


  //  cam = new Camera ;
  cam.open(mode,fps,input, sp)  ;


  init=true ;

  cam.start() ;

}


#endif
