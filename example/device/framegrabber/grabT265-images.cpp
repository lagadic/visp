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
 * Images acquisition with RealSense T265 sensor and librealsense2.
 *
 *****************************************************************************/

/*!
  \example grabT265-images.cpp
  This example shows how to retrieve images from a RealSense T265 sensor with
  librealsense2.
*/

#include <iostream>

#include <visp3/core/vpImageConvert.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/sensor/vpRealSense2.h>

#if defined(VISP_HAVE_REALSENSE2) && (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11) && \
    (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI))

int main()
{
  try {
    double ts;
    vpRealSense2 rs;
    rs2::config config;
    config.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
    config.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);

    rs.open(config);

    // Creating left and right vpImages
    vpImage<unsigned char> Il((unsigned int)rs.getIntrinsics(RS2_STREAM_FISHEYE).height,
                              (unsigned int)rs.getIntrinsics(RS2_STREAM_FISHEYE).width);

    vpImage<unsigned char> Ir((unsigned int)rs.getIntrinsics(RS2_STREAM_FISHEYE).height,
                              (unsigned int)rs.getIntrinsics(RS2_STREAM_FISHEYE).width);

#if defined(VISP_HAVE_X11)
    vpDisplayX fe_l(Il, 10, 10, "Left image"); // Left
    vpDisplayX fe_r(Ir, (int)Il.getWidth() + 80, 10, "Right image"); // Right
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI fe_l(Il, 10, 10, "Right image");
    vpDisplayGDI fe_r(Ir, Il.getWidth() + 80, 10, "Left image");
#endif

    while (true) {
      double t = vpTime::measureTimeMs();

      // Acquire both images with timestamp
      rs.acquire(&Il, &Ir, &ts);

      vpDisplay::display(Il);
      vpDisplay::display(Ir);

      vpDisplay::displayText(Il, 15, 15, "Click to quit", vpColor::red);
      vpDisplay::displayText(Ir, 15, 15, "Click to quit", vpColor::red);

      if (vpDisplay::getClick(Il, false) || vpDisplay::getClick(Ir, false)) {
        break;
      }
      vpDisplay::flush(Il);
      vpDisplay::flush(Ir);

      std::cout << "Loop time: " << vpTime::measureTimeMs() - t << std::endl;
    }

  } catch (const vpException &e) {
    std::cerr << "RealSense error " << e.what() << std::endl;
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
  }

  return EXIT_SUCCESS;
}
#endif