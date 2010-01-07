/****************************************************************************
 *
 * $Id$
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
 * Frame grabbing.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/



#ifndef vpFrameGrabber_hh
#define vpFrameGrabber_hh


#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vpRGBa.h>

/*!
  \file vpFrameGrabber.h
  \brief Base class for all video devices. It is
         designed to provide a generic front end to video sources.
*/

/*!
  \class vpFrameGrabber

  \ingroup Framegrabber

  \brief Base class for all video devices. It is designed to provide a front
  end to video sources.

  This class should provide a virtual function that allows the acquisition
  of an image.

  The example below shows how to use this class.
  \code
#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpV4l2Grabber.h>
#include <visp/vp1394TwoGrabber.h>

int main()
{
#if defined(VISP_HAVE_V4L2) || defined(VISP_HAVE_DC1394_2)
  vpImage<unsigned char> I;
  vpFrameGrabber *g; // Generic framegrabber

#if defined( VISP_HAVE_DC1394_2 )
  vp1394TwoGrabber *g_1394_2 = new vp1394TwoGrabber;
  // specific settings for firewire grabber based on libdc1394-2.x version
  g_1394_2->setVideoMode(vp1394TwoGrabber::vpVIDEO_MODE_320x240_YUV422);
  g_1394_2->setFramerate(vp1394TwoGrabber::vpFRAMERATE_30);
  g = g_1394_2;
#elif defined( VISP_HAVE_V4L2 )
  vpV4l2Grabber *g_v4l2 = new vpV4l2Grabber;
  // specific settings for Video For Linux Two grabber
  g_v4l2->setInput(2);    // Input 2 on the board
  g_v4l2->setFramerate(vpV4l2Grabber::framerate_50fps); // 50 fps
  g_v4l2->setWidth(384);  // Acquired images are 768 width
  g_v4l2->setHeight(288); // Acquired images are 576 height
  g_v4l2->setNBuffers(3); // 3 ring buffers to ensure real-time acquisition
  g = g_v4l2;
#endif

  g->open(I);                           // Open the framegrabber
  g->acquire(I);                        // Acquire an image
  vpImageIo::writePGM(I, "image.pgm");  // Write image on the disk
#endif
}
  \endcode


  \author Eric Marchand (Eric.Marchand@irisa.fr), Irisa / Inria Rennes
*/
class VISP_EXPORT vpFrameGrabber
{
public :
  bool   init ;  //!< bit 1 if the frame grabber has been initialized
protected:
  unsigned int height ;  //!< number of rows in the image
  unsigned int width ;  //!< number of columns in the image


public:
  //! return the number of rows in the image
  inline  unsigned int getHeight() { return height ; }
  //! return the number of columns in the image
  inline  unsigned int getWidth() { return width ; }

public:
  virtual ~vpFrameGrabber() { ; }

  virtual void open(vpImage<unsigned char> &I) =0 ;
  virtual void open(vpImage<vpRGBa> &I) =0 ;

  virtual void acquire(vpImage<unsigned char> &I) =0 ;
  virtual void acquire(vpImage<vpRGBa> &I) =0 ;


  /*!
    This virtual function is used to de-allocate
    the memory used by a specific frame grabber
  */
  virtual void close() =0 ;

} ;

#endif
