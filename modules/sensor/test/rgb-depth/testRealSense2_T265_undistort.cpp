/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
 * See https://visp.inria.fr for more information.
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
 * Image acquisition with RealSense T265 sensor and librealsense2 and
 * undistorting it using vpImageTools.
 */

/*!
  \example testRealSense2_T265_undistort.cpp
  This example shows how to retrieve single image from a RealSense T265 device
  with librealsense2. Undistorting the image is done using vpImageTools.
*/

#include <iostream>

#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpImageTools.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/sensor/vpRealSense2.h>

#if defined(VISP_HAVE_REALSENSE2) && (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11) && \
    (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI)) && (RS2_API_VERSION > ((2 * 10000) + (31 * 100) + 0))

int main()
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  try {
    vpCameraParameters cam_left;
    unsigned int display_scale = 2;
    vpRealSense2 rs;
    std::string product_line = rs.getProductLine();
    std::cout << "Product line: " << product_line << std::endl;

    if (product_line != "T200") {
      std::cout << "This example doesn't support devices that are not part of T200 product line family !" << std::endl;
      return EXIT_SUCCESS;
    }
    int cam_index = 1; // Left fisheye camera
    // Both streams should be enabled.
    // Note: It is not currently possible to enable only one
    rs2::config config;
    config.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
    config.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);
    rs.open(config);
    cam_left =
      rs.getCameraParameters(RS2_STREAM_FISHEYE, vpCameraParameters::ProjWithKannalaBrandtDistortion, cam_index);

    vpImage<unsigned char> I((unsigned int)rs.getIntrinsics(RS2_STREAM_FISHEYE).height,
                             (unsigned int)rs.getIntrinsics(RS2_STREAM_FISHEYE).width);

    vpImage<unsigned char> I_undist((unsigned int)rs.getIntrinsics(RS2_STREAM_FISHEYE).height,
                                    (unsigned int)rs.getIntrinsics(RS2_STREAM_FISHEYE).width);

    std::cout << "Left fisheye camera parameters: " << cam_left << std::endl;

#if defined(VISP_HAVE_X11)
    vpDisplayX d;
    vpDisplayX d_undist;
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d;
    vpDisplayGDI d_undist;
#endif

#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI)
    d.setDownScalingFactor(display_scale);
    d_undist.setDownScalingFactor(display_scale);
    d.init(I, 10, 10, "Left image");
    d_undist.init(I_undist, I.getWidth() / display_scale + 80, 10, "Undistorted image");
#endif

    vpArray2D<int> mapU, mapV;
    vpArray2D<float> mapDu, mapDv;

    vpImageTools::initUndistortMap(cam_left, I.getWidth(), I.getHeight(), mapU, mapV, mapDu, mapDv);

    while (true) {
      double t = vpTime::measureTimeMs();

      rs.acquire(&I, nullptr, nullptr); // Acquire only left image

      vpDisplay::display(I);

      vpImageTools::undistort(I, mapU, mapV, mapDu, mapDv, I_undist);
      vpDisplay::display(I_undist);

      vpDisplay::displayText(I, 15 * display_scale, 15 * display_scale, "Click to quit", vpColor::red);
      vpDisplay::displayText(I_undist, 15 * display_scale, 15 * display_scale, "Click to quit", vpColor::red);

      if (vpDisplay::getClick(I, false) || vpDisplay::getClick(I_undist, false))
        break;

      vpDisplay::flush(I);
      vpDisplay::flush(I_undist);

      std::cout << "Loop time: " << vpTime::measureTimeMs() - t << std::endl;
    }

  }
  catch (const vpException &e) {
    std::cerr << "RealSense error " << e.what() << std::endl;
  }
  catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
  }

  return EXIT_SUCCESS;
}
#else
int main()
{
#if !defined(VISP_HAVE_REALSENSE2)
  std::cout << "You do not realsense2 SDK functionality enabled..." << std::endl;
  std::cout << "Tip:" << std::endl;
  std::cout << "- Install librealsense2, configure again ViSP using cmake and build again this example" << std::endl;
  return EXIT_SUCCESS;
#elif !(defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI))
  std::cout << "You don't have X11 or GDI display capabilities" << std::endl;
#elif !(RS2_API_VERSION > ((2 * 10000) + (31 * 100) + 0))
  std::cout << "Install librealsense version > 2.31.0" << std::endl;
#endif
  return EXIT_SUCCESS;
}
#endif
