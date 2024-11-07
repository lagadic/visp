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
 * IMU data acquisition with Structure Core sensor and libStructure.
 */

/*!
  \example testOccipitalStructure_Core_imu.cpp
  This example shows how to retrieve IMU data from a Occipital Structure Core sensor
  with libStructure.
*/

#include <iostream>

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_OCCIPITAL_STRUCTURE) && (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)

#include <visp3/gui/vpPlot.h>
#include <visp3/sensor/vpOccipitalStructure.h>

int main()
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  try {
    double initial_ts, ts;
    vpOccipitalStructure sc;
    vpColVector imu_vel, imu_acc;

    ST::CaptureSessionSettings settings;
    settings.source = ST::CaptureSessionSourceId::StructureCore;
    settings.structureCore.depthEnabled = true;
    settings.structureCore.visibleEnabled = true;
    settings.structureCore.accelerometerEnabled = true;
    settings.structureCore.gyroscopeEnabled = true;
    settings.structureCore.imuUpdateRate = ST::StructureCoreIMUUpdateRate::AccelAndGyro_1000Hz;

    sc.open(settings);

    // Create plotters
    vpPlot acceleration_plotter(1, 400, 800, 10, 10, "Accelerations");
    vpPlot rotationRate_plotter(1, 400, 800, 10, 450, "Rotation rates");
    acceleration_plotter.initGraph(0, 3);
    rotationRate_plotter.initGraph(0, 3);

    acceleration_plotter.setColor(0, 0, vpColor::blue);
    acceleration_plotter.setColor(0, 1, vpColor::red);
    acceleration_plotter.setColor(0, 2, vpColor::green);
    rotationRate_plotter.setColor(0, 0, vpColor::blue);
    rotationRate_plotter.setColor(0, 1, vpColor::red);
    rotationRate_plotter.setColor(0, 2, vpColor::green);

    acceleration_plotter.plot(0, 0, 0, 0);
    acceleration_plotter.plot(0, 1, 0, 0);
    acceleration_plotter.plot(0, 2, 0, 0);
    rotationRate_plotter.plot(0, 0, 0, 0);
    rotationRate_plotter.plot(0, 1, 0, 0);
    rotationRate_plotter.plot(0, 2, 0, 0);

    acceleration_plotter.setLegend(0, 0, "X axis");
    acceleration_plotter.setLegend(0, 1, "Y axis");
    acceleration_plotter.setLegend(0, 2, "Z axis");
    rotationRate_plotter.setLegend(0, 0, "X axis");
    rotationRate_plotter.setLegend(0, 1, "Y axis");
    rotationRate_plotter.setLegend(0, 2, "Z axis");

    sc.getIMUData(&imu_vel, &imu_acc, &initial_ts);

    bool quit = false;
    while (!quit) {
      sc.getIMUData(&imu_vel, &imu_acc, &ts);

      acceleration_plotter.plot(0, ts - initial_ts, imu_acc);
      rotationRate_plotter.plot(0, ts - initial_ts, imu_vel);
      if (vpDisplay::getClick(acceleration_plotter.I, false) || vpDisplay::getClick(rotationRate_plotter.I, false)) {
        quit = true;
      }
    }
  }
  catch (const vpException &e) {
    std::cerr << "Structure SDK error " << e.what() << std::endl;
  }
  catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
  }

  return EXIT_SUCCESS;
}
#else
int main()
{
#if !defined( VISP_HAVE_OCCIPITAL_STRUCTURE )
  std::cout << "You do not have Occipital Structure SDK functionality enabled..." << std::endl;
  std::cout << "Tip:" << std::endl;
  std::cout << "- Install libStructure, configure again ViSP using cmake and build again this example" << std::endl;
  return EXIT_SUCCESS;
#elif ( VISP_CXX_STANDARD < VISP_CXX_STANDARD_11 )
  std::cout << "You do not build ViSP with c++11 or higher compiler flag" << std::endl;
  std::cout << "Tip:" << std::endl;
  std::cout << "- Configure ViSP again using cmake -DUSE_CXX_STANDARD=11, and build again this example" << std::endl;
#endif
  return EXIT_SUCCESS;
}
#endif
