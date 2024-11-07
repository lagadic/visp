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
 * Acquisition of IMU data with RealSense T265 sensor and librealsense2.
 */

/*!
  \example testRealSense2_T265_imu.cpp
  This example shows how to retrieve imu acceleration and velocity data
  from a RealSense T265 sensor and librealsense2.

*/

#include <iostream>

#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/sensor/vpRealSense2.h>

#if defined(VISP_HAVE_REALSENSE2) && (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11) && \
    (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI)) && (RS2_API_VERSION > ((2 * 10000) + (31 * 100) + 0))

int main()
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  vpColVector imu_acc, imu_vel;

  try {
    vpRealSense2 rs;
    std::string product_line = rs.getProductLine();
    std::cout << "Product line: " << product_line << std::endl;

    if (product_line != "T200") {
      std::cout << "This example doesn't support devices that are not part of T200 product line family !" << std::endl;
      return EXIT_SUCCESS;
    }
    rs2::config config;
    config.enable_stream(RS2_STREAM_ACCEL);
    config.enable_stream(RS2_STREAM_GYRO);
    rs.open(config);

    while (true) {

      rs.getIMUData(&imu_acc, &imu_vel, nullptr);

      std::cout << "Gyro vel: x = " << std::setw(12) << imu_vel[0] << " y = " << std::setw(12) << imu_vel[1]
        << " z = " << std::setw(12) << imu_vel[2];
      std::cout << " Accel: x = " << std::setw(12) << imu_acc[0] << " y = " << std::setw(12) << imu_acc[1]
        << " z = " << std::setw(12) << imu_acc[2];
      std::cout << std::endl;
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
