/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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

#include <stdio.h>
#include <stdlib.h>
#include <visp3/core/vpImage.h>
#include <visp3/sensor/vp1394TwoGrabber.h>

int main()
{
#ifdef VISP_HAVE_DC1394
  try {
    unsigned int ncameras; // Number of cameras on the bus
    vp1394TwoGrabber g;
    g.getNumCameras(ncameras);
    vpImage<unsigned char> *I = new vpImage<unsigned char>[ncameras];

    // If the first camera supports vpVIDEO_MODE_640x480_YUV422 video mode
    g.setCamera(0);
    g.setVideoMode(vp1394TwoGrabber::vpVIDEO_MODE_640x480_YUV422);

    // If all cameras support 30 fps acquisition
    for (unsigned int camera = 0; camera < ncameras; camera++) {
      g.setCamera(camera);
      g.setFramerate(vp1394TwoGrabber::vpFRAMERATE_30);
    }

    for (;;) {
      for (unsigned int camera = 0; camera < ncameras; camera++) {
        // Acquire successively images from the different cameras
        g.setCamera(camera);
        g.acquire(I[camera]);
      }
    }
    delete[] I;
    return EXIT_SUCCESS;
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }

#endif
}
