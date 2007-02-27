/****************************************************************************
 *
 * $Id: vpOSXcfoxGrabber.cpp,v 1.8 2007-02-27 17:08:05 fspindle Exp $
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit
 *
 * This file may be distributed under the terms of the Q Public License
 * as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE included in the packaging of this file.
 *
 * Licensees holding valid ViSP Professional Edition licenses may
 * use this file in accordance with the ViSP Commercial License
 * Agreement provided with the Software.
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
 *
 * Description:
 * 1394 frame grabbing for OSX.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/



/*!
  \file vpOSXcfoxGrabber.cpp
  \brief class for the Video For Linux 2 video device
  \ingroup libdevice
*/

#include <visp/vpConfig.h>

#if ( defined(APPLE) && defined(VISP_HAVE_CFOX) )

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
#include <visp/vpImageConvert.h>


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
vpOSXcfoxGrabber::setInput(unsigned int input)
{
  this->input = input;
}

/*!
  Set the decimation factor.

  \exception vpFrameGrabberException::settingError : Wrong scale (shoud be
  between 1 and 16).

  \param scale : Decimation factor.
*/
void
vpOSXcfoxGrabber::setScale(unsigned int scale)
{
  if ((scale <1) || (scale >2))
  {
    close();

    vpERROR_TRACE("Wrong scale %d, scale shoud be between 1 and 2",scale) ;
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "Wrong scale") );
  }

  this->scale = scale ;
}

/*!
  Initialize image acquisition.

  \param I : Image data structure (8 bits image)

  \exception vpFrameGrabberException::settingError : Wrong input channel.

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

  \exception vpFrameGrabberException::initializationError : Frame grabber not
  initialized.

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
       vpImageConvert::YUV422ToGrey(img,(unsigned char *)I.bitmap,I.getWidth()*I.getHeight()) ;

    }

}


/*!
  Acquire a color image.

  \param I : Image data structure (32 bits image)

  \exception vpFrameGrabberException::initializationError : Frame grabber not
  initialized.

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
      vpImageConvert::YUV422ToRGBa(img,(unsigned char *)I.bitmap,I.getWidth()*I.getHeight()) ;

    }

}

/*!

  Set the framerate of the acquisition.

  \param _framerate : The framerate for the acquisition.

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

  \exception vpFrameGrabberException::initializationError : Can't access to
  video device.

  \exception vpFrameGrabberException::otherError : Can't query video
  capabilities.

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
