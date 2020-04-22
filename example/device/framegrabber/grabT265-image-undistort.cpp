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
 * Image acquisition with RealSense T265 sensor and librealsense2 and 
 * undistorting it using vpImageTools
 *
 *****************************************************************************/

/*!
  \example grabT265-image-undistort.cpp
  This example shows how to retrieve single image from a RealSense T265 device
  with librealsense2. Undistorting the image is done using vpImageTools.
*/

#include <iostream>

#include <opencv2/calib3d/calib3d.hpp>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpImageTools.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/sensor/vpRealSense2.h>

#if defined(VISP_HAVE_REALSENSE2) && (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11) && \
    (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI))

int main()
{
  try {
    vpCameraParameters cam_L;
    vpRealSense2 rs;
    int cam_index = 1;
    // Both streams should be enabled. 
    // Note: It is not currently possible to enable only one
    rs2::config config;
    config.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
    config.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);
    
    rs.open(config);
    cam_L = rs.getCameraParameters(RS2_STREAM_FISHEYE, vpCameraParameters::perspectiveProjWithDistortion, cam_index);

    vpImage<unsigned char> I((unsigned int)rs.getIntrinsics(RS2_STREAM_FISHEYE).height,
                              (unsigned int)rs.getIntrinsics(RS2_STREAM_FISHEYE).width);
    
    vpImage<unsigned char> undistI((unsigned int)rs.getIntrinsics(RS2_STREAM_FISHEYE).height,
                              (unsigned int)rs.getIntrinsics(RS2_STREAM_FISHEYE).width);
    

    cam_L.printParameters();
    std::cout << std::endl;
    
#if defined(VISP_HAVE_X11)
    vpDisplayX d(I, 10, 10, "Left image"); // Left
    vpDisplayX d1(undistI, I.getWidth(), 10, "Undistorted image"); // Left
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI fe_l(I, 10, 10, "Right image");
#endif

    vpArray2D<int> mapU, mapV;
    vpArray2D<float> mapDu, mapDv;

    while (true) {
      double t = vpTime::measureTimeMs();

      rs.acquire(&I, NULL, NULL); // Acquire only left image

      vpDisplay::display(I);

      vpImageTools::undistortFisheye(I, cam_L, undistI);
      vpDisplay::display(undistI);

      vpDisplay::displayText(I, 15, 15, "Click to quit", vpColor::red);
      
      if (vpDisplay::getClick(I, false))
        break;

      vpDisplay::flush(I);
      vpDisplay::flush(undistI);

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
