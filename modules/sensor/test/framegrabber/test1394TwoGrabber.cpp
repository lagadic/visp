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
 * Firewire cameras video capture.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \file test1394TwoGrabber.cpp

  \brief  Aquire images using libdc1394-2.x library.

*/

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h>

#include <iostream>
#include <string>

#if defined(VISP_HAVE_DC1394)

#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/sensor/vp1394TwoGrabber.h>
/*!
  \example test1394TwoGrabber.cpp



*/
int main()
{
  try {
    std::cout << "IEEE1394 test in progress..." << std::endl;

    // Get the user name
    std::string username;
    vpIoTools::getUserName(username);
    std::string outputpath = "/tmp/" + username;
    vpIoTools::makeDirectory(outputpath);

    // Creation of an empty image container
    vpImage<unsigned char> I;

    // Creation of a framegrabber
    vp1394TwoGrabber g;
    uint64_t guid;

    // Get the number of cameras connected on the bus
    unsigned int ncameras; // Number of cameras on the bus
    ncameras = g.getNumCameras();
    for (unsigned int i = 0; i < ncameras; i++) {
      g.setCamera(i);
      g.getGuid(guid);
      std::cout << "Detected camera with GUID 0x" << std::hex << guid << std::endl;
    }
    // If more than one camera connected, use the last one
    if (ncameras > 1) {
      g.setCamera(ncameras - 1);
      g.getGuid(guid);
      std::cout << "Use camera with GUID 0x" << std::hex << guid << std::endl;
      g.setCamera(0); // to be sure that the setCamera() in the next line with
                      // guid as parameter works
      g.setCamera(guid);
    }

    for (int i = 0; i < 10; i++)
      g.acquire(I);
    g.close();
    std::string filename = outputpath + "/imagetest1.pgm";
    std::cout << "Write image: " << filename << std::endl;
    vpImageIo::write(I, filename);

    std::cout << "New connection..." << std::endl;
    g.open(I);
    g.close();

    std::cout << "New connection..." << std::endl;
    g.open(I);
    // g.setVideoMode(vp1394TwoGrabber::vpVIDEO_MODE_800x600_YUV422);
    g.acquire(I);
    g.close();
    filename = outputpath + "/imagetest2.pgm";
    std::cout << "Write image: " << filename << std::endl;
    vpImageIo::write(I, filename);
  } catch (...) {
    vpCERROR << "Failure: exit" << std::endl;
  }
}
#else
int main()
{
  vpTRACE("Ieee 1394 grabber capabilities are not available...\n"
          "You should install libdc1394-2 to use this binary.");
}

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
