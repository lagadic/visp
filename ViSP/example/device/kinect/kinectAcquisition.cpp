/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2012 by INRIA. All rights reserved.
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
 * Kinect example.
 *
 * Authors:
 * C�line Teuli�re
 *
 *****************************************************************************/


/*!
  \example kinectAcquisition.cpp

  \brief Example that shows how to acquire depth map and RGB images from a kinect device.

*/


#include <visp/vpConfig.h>
#include <iostream>
#ifdef VISP_HAVE_LIBFREENECT_AND_DEPENDENCIES

#if (defined (VISP_HAVE_X11) || defined(VISP_HAVE_GTK) || defined(VISP_HAVE_OPENCV) || defined(VISP_HAVE_GDI))	


#include <visp/vpImage.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpKinect.h>
#include <visp/vpTime.h>

int main() {
  // Init Kinect
#ifdef VISP_HAVE_LIBFREENECT_OLD
  // This is the way to initialize Freenect with an old version of libfreenect packages under ubuntu lucid 10.04
  Freenect::Freenect<vpKinect> freenect;
  vpKinect & kinect = freenect.createDevice(0);
#else
  Freenect::Freenect freenect;
  vpKinect & kinect = freenect.createDevice<vpKinect>(0);
#endif

  // Set tilt angle in degrees
  if (0) {
    float angle = -3;
    kinect.setTiltDegrees(angle);
  }

  // Init display
#if 1
  kinect.start(vpKinect::DMAP_MEDIUM_RES); // Start acquisition thread with a depth map resolution of 480x640
  vpImage<unsigned char> Idmap(480,640);//for medium resolution
  vpImage<float> dmap(480,640);//for medium resolution
#else
  kinect.start(vpKinect::DMAP_LOW_RES); // Start acquisition thread with a depth map resolution of 240x320 (default resolution)
  vpImage<unsigned char> Idmap(240,320);//for low resolution
  vpImage<float> dmap(240,320);//for low resolution
#endif
  vpImage<vpRGBa> Irgb(480,640);

#if defined VISP_HAVE_X11
  vpDisplayX display;
  vpDisplayX displayRgb;
#elif defined VISP_HAVE_GTK
  vpDisplayGTK display;
  vpDisplayGTK displayRgb;
#elif defined VISP_HAVE_OPENCV
  vpDisplayOpenCV display;
  vpDisplayOpenCV displayRgb;
#elif defined VISP_HAVE_GDI
  vpDisplayGDI display;
  vpDisplayGDI displayRgb;
#endif

  display.init(Idmap, 100, 200,"Depth map");
  displayRgb.init(Irgb, 900, 200,"Color Image");

  // A click to stop acquisition
  std::cout << "Click in one image to stop acquisition" << std::endl;

  while(!vpDisplay::getClick(Idmap,false) && !vpDisplay::getClick(Irgb,false))
    {
      kinect.getDepthMap(dmap);
      kinect.getDepthMap(dmap, Idmap);
      kinect.getRGB(Irgb);

      vpDisplay::display(Idmap);
      vpDisplay::flush(Idmap);
      vpDisplay::display(Irgb);
      vpDisplay::flush(Irgb);
     }
  std::cout << "Stop acquisition" << std::endl;
  kinect.stop(); // Stop acquisition thread
  return 0;
}

#else

int
main()
{
  std::cout << "You should install a video device (X11, GTK, OpenCV, GDI) to run this example" << std::endl;
}
#endif

#else
int
main()
{
  std::cout << "You should install libfreenect to run this example" << std::endl;
}

#endif


