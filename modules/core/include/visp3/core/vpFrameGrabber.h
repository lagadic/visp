/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
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

#include <visp3/core/vpImage.h>
#include <visp3/core/vpRGBa.h>

/*!
  \file vpFrameGrabber.h
  \brief Base class for all video devices. It is
         designed to provide a generic front end to video sources.
*/

/*!
  \class vpFrameGrabber

  \brief Base class for all video devices. It is designed to provide a front
  end to video sources.

  This class should provide a virtual function that allows the acquisition
  of an image.

  The example below shows how to use this class.
  \code
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/sensor/vp1394TwoGrabber.h>
#include <visp3/sensor/vpV4l2Grabber.h>

int main()
{
#if defined(VISP_HAVE_V4L2) || defined(VISP_HAVE_DC1394)
  vpImage<unsigned char> I;
  vpFrameGrabber *g; // Generic framegrabber

#if defined( VISP_HAVE_DC1394 )
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
  vpImageIo::write(I, "image.pgm");  // Write image on the disk
#endif
}
  \endcode


  \author Eric Marchand (Eric.Marchand@irisa.fr), Irisa / Inria Rennes
*/
class VISP_EXPORT vpFrameGrabber
{
public:
  bool init; //!< Set to true if the frame grabber has been initialized.

protected:
  unsigned int height; //!< Number of rows in the image.
  unsigned int width;  //!< Number of columns in the image.

public:
  /** @name Inherited functionalities from vpFramegrabber */
  //@{
  //! Return the number of rows in the image.
  inline unsigned int getHeight() const { return height; }
  //! Return the number of columns in the image.
  inline unsigned int getWidth() const { return width; }
  //@}

public:
  vpFrameGrabber() : init(false), height(0), width(0){};
  virtual ~vpFrameGrabber() { ; }

  virtual void open(vpImage<unsigned char> &I) = 0;
  virtual void open(vpImage<vpRGBa> &I) = 0;

  virtual void acquire(vpImage<unsigned char> &I) = 0;
  virtual void acquire(vpImage<vpRGBa> &I) = 0;

  /*!
    This virtual function is used to de-allocate
    the memory used by a specific frame grabber
  */
  virtual void close() = 0;
};

#endif
