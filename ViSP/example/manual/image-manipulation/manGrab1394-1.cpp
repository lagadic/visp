/****************************************************************************
 *
 * $Id: manGrab1394-1.cpp,v 1.5 2008-06-17 08:43:18 asaunier Exp $
 *
 * Copyright (C) 1998-2008 Inria. All rights reserved.
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
 * Images grabbing example.
 *
 * Authors:
 * Anthony Saunier
 * Fabien Spindler
 *
 *****************************************************************************/
/*!
  \file manGrab1394-1.cpp

  \brief Images grabbing example with the vp1394Grabber class.

 */
/*!
  \example manGrab1394-1.cpp

  \brief Images grabbing example with the vp1394Grabber class.
 
 */

#include <visp/vpConfig.h>

#include <stdlib.h>
#include <stdio.h>
#include <visp/vpImage.h>
#include <visp/vp1394Grabber.h>

int main()
{
#ifdef VISP_HAVE_DC1394_1
  unsigned int ncameras; // Number of cameras on the bus
  vp1394Grabber g;
  g.getNumCameras(ncameras);
  vpImage<unsigned char> *I = new vpImage<unsigned char> [ncameras];
  
  // If the first camera supports MODE_640x480_YUV422 video mode
  g.setCamera(0);
  g.setFormat(FORMAT_VGA_NONCOMPRESSED);
  g.setMode(MODE_640x480_YUV422);
  
  // If all cameras support 30 fps acquisition
  for (unsigned int camera=0; camera < ncameras; camera ++) {
    g.setCamera(camera);
    g.setFramerate(FRAMERATE_30);
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
