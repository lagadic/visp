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
 * Basler cameras video capture using Pylon SDK.
 *
 * Authors:
 * Wenfeng CAI
 *
 *****************************************************************************/

/*!
  \file testPylonGrabber.cpp

  \brief  Acquire images using Pylon library.
*/

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h>

#include <iostream>
#include <string>

#if defined(VISP_HAVE_PYLON)

#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/sensor/vpPylonFactory.h>
/*!
  \example testPylonGrabber.cpp
*/
int main()
{
  try {
    std::cout << "Basler camera test with Pylon in progress..." << std::endl;

    // Get the user name
    std::string username;
    vpIoTools::getUserName(username);
    std::string outputpath = "/tmp/" + username;
    vpIoTools::makeDirectory(outputpath);

    // Creation of an empty image container
    vpImage<unsigned char> I;

    vpPylonFactory &factory = vpPylonFactory::instance();
    // Creation of a framegrabber
    vpPylonGrabber *g = factory.createPylonGrabber(vpPylonFactory::BASLER_GIGE);
    std::string guid;

    // Get the number of cameras connected on the bus
    unsigned int ncameras; // Number of cameras on the bus
    ncameras = g->getNumCameras();
    for (unsigned int i = 0; i < ncameras; i++) {
      g->setCameraIndex(i);
      guid = g->getCameraSerial(i);
      std::cout << "Detected camera with serial: " << guid << std::endl;
    }
    // If more than one camera connected, use the first one
    if (ncameras > 1) {
      g->setCameraIndex(0);
      guid = g->getCameraSerial(0);
      std::cout << "Use camera with serial: " << guid << std::endl;
      // to be sure that the setCamera() in the next line with
      // guid as parameter works
      g->setCameraIndex(0);
      g->setCameraSerial(guid);
    }
    g->getCameraInfo(std::cout);

    std::cout << "Frame rate: " << g->getFrameRate() << std::endl;
    std::cout << "Gain: " << g->getGain() << std::endl;
    std::cout << "Gamma: " << g->getGamma() << std::endl;
    std::cout << "Exposure time (ms): " << g->getExposure() << std::endl;
    float blackLevel = g->getBlackLevel();
    std::cout << "Black level: " << blackLevel << std::endl;

    for (int i = 0; i < 10; i++)
      g->acquire(I);
    g->close();
    std::cout << "Current image size: " << g->getWidth() << "x" << g->getHeight() << std::endl;

    std::string filename = outputpath + "/imagetest1.pgm";
    std::cout << "Write image: " << filename << std::endl;
    vpImageIo::write(I, filename);

    std::cout << "New connection..." << std::endl;
    g->open(I);
    g->close();

    std::cout << "New connection..." << std::endl;
    g->open(I);
    g->close();
    filename = outputpath + "/imagetest2.pgm";
    std::cout << "Write image: " << filename << std::endl;
    vpImageIo::write(I, filename);
  } catch (const vpException &e) {
    vpCERROR << e.what() << std::endl;
  } catch (const std::exception &e) {
    vpCERROR << e.what() << std::endl;
  } catch (...) {
    vpCERROR << "Failure: exit" << std::endl;
  }
}
#else
int main()
{
  vpTRACE("Basler Pylon grabber capabilities are not available...\n"
          "You should install pylon SDK to use this binary.");
}

#endif
