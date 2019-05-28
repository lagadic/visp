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
 * Test for Virtuose + Glove SDK wrapper.
 *
 * Author:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \example testVirtuoseWithGlove.cpp
  Test for reading the Virtuose's and Glove joint positions.

  Usage: ./testVirtuoseWithGlove
*/

#include <stdlib.h>

#include <iostream>
#include <sstream>
#include <vector>

#include <visp3/robot/vpVirtuose.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayGDI.h>

#if defined(VISP_HAVE_VIRTUOSE) && (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI))

int main()
{
  int port = 5000;

  std::vector<vpVirtuose> virtuose(4); // 0: virtuose, 1: thumb, 2: index, 3: middle
  std::vector<vpHomogeneousMatrix> wMd(4);

  // Position of the external camera with respect of the virtuose world frame
  vpHomogeneousMatrix wMc(0.3, 0, 1.2, vpMath::rad(180), 0, 0);

  // Open device
  for (size_t device=0; device < virtuose.size(); device ++) {
    std::stringstream ss; ss << port + device;
    std::string ip = "localhost#" + ss.str();
    std::cout << "Connect to: " << ip << std::endl;

    virtuose[device].setIpAddress(ip);
    virtuose[device].init();
  }

  std::cout << "After init" << std::endl;
#if 0
  // Get joint position
  for (size_t device=0; device < virtuose.size(); device ++) {
    std::cout << "Number of joints: " << virtuose[device].getJointsNumber()
              << " Joint position: " << virtuose[device].getArticularPosition().t() << std::endl
              << " Cartesian position: " << virtuose[device].getAvatarPosition().t() << std::endl;
    //    std::cout << "Joint velocity: " << virtuose.getArticularVelocity().t() << std::endl;
  }
#endif
  bool end = false;
  vpImage<unsigned char> I(480, 640, 0);
  vpCameraParameters cam;

#if defined(VISP_HAVE_X11)
  vpDisplayX d(I);
#elif defined(VISP_HAVE_GDI)
  vpDisplayGDI d(I);
#endif

  while(!end) {
    vpDisplay::display(I);
    // Get joint position
    for (size_t device=0; device < virtuose.size(); device ++) {
      vpPoseVector wpd = virtuose[device].getAvatarPosition();
      std::cout << "Device #" << device << " has position: " << wpd.t() << std::endl;
      wMd[device].buildFrom(wpd);

      vpHomogeneousMatrix cMd = wMc.inverse() * wMd[device];
      vpDisplay::displayFrame(I, cMd, cam, 0.1);

      std::stringstream ss;
      ss << device;
      double X = cMd[0][3];
      double Y = cMd[1][3];
      double Z = cMd[2][3];
      vpImagePoint pos;
      vpMeterPixelConversion::convertPoint(cam, X/Z, Y/Z, pos);
      vpDisplay::displayText(I, pos + vpImagePoint(10, 10), ss.str(), vpColor::white);
      vpDisplay::displayText(I, 10, 10, "Click to quit...", vpColor::red);
    }
    if(vpDisplay::getClick(I, false)) {
      end = true;
    }
    vpDisplay::flush(I);

  }

  // Close device
  for (size_t device=0; device < virtuose.size(); device ++) {
    virtuose[device].close();
  }
  std::cout << "The end" << std::endl;
}

#else
int main() { std::cout << "You should install Virtuose API to use this binary..." << std::endl; }
#endif
