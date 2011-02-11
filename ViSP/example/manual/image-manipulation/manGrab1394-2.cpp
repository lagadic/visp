/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2011 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * Description:
 * Images grabbing example.
 *
 * Authors:
 * Anthony Saunier
 * Fabien Spindler
 *
 *****************************************************************************/
/*!
  \file manGrab1394-2.cpp

  \brief Images grabbing example with the vp1394TwoGrabber class.

 */
/*!
  \example manGrab1394-2.cpp

  \brief Images grabbing example with the vp1394TwoGrabber class.
 
 */

#include <stdlib.h>
#include <stdio.h>

#include <visp/vpImage.h>
#include <visp/vp1394TwoGrabber.h>

int main()
{
#ifdef VISP_HAVE_DC1394_2

  unsigned int ncameras; // Number of cameras on the bus
  vp1394TwoGrabber g;
  g.getNumCameras(ncameras);
  vpImage<unsigned char> *I = new vpImage<unsigned char> [ncameras];
  
  // If the first camera supports vpVIDEO_MODE_640x480_YUV422 video mode
  g.setCamera(0);
  g.setVideoMode(vp1394TwoGrabber::vpVIDEO_MODE_640x480_YUV422);
  
  // If all cameras support 30 fps acquisition
  for (unsigned int camera=0; camera < ncameras; camera ++) {
    g.setCamera(camera);
    g.setFramerate(vp1394TwoGrabber::vpFRAMERATE_30);
  }
  
  while(1) {
    for (unsigned int camera=0; camera < ncameras; camera ++) {
      // Acquire successively images from the different cameras
      g.setCamera(camera);
      g.acquire(I[camera]);
    }
  }
  delete [] I;

#endif

  return 0;
}
