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
 * Interface for the Irisa's Afma6 robot.
 */

/*!
  \example testRobotBebop2.cpp

  Example to control Parrot Bebop2.
*/

#include <iostream>

#include <visp3/core/vpTime.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/robot/vpRobotBebop2.h>

int main(int argc, char **argv)
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
#ifdef VISP_HAVE_ARSDK
  try {
    int stream_res = 0;
    std::string ip_address = "192.168.42.1";
    bool verbose = false;

    for (int i = 1; i < argc; i++) {
      if (std::string(argv[i]) == "--ip" && i + 1 < argc) {
        ip_address = std::string(argv[i + 1]);
        i++;
      }
      else if (std::string(argv[i]) == "--hd-resolution") {
        stream_res = 1;
      }
      else if (std::string(argv[i]) == "--verbose" || std::string(argv[i]) == "-v") {
        verbose = true;
      }
      else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
        std::cout << "\nUsage:\n"
          << "  " << argv[0] << " [--ip <drone ip>] [--hd-resolution] [--verbose] [-v]"
          << " [--help] [-h]\n"
          << std::endl
          << "Description:\n"
          << "  --ip <drone ip>\n"
          << "     IP address of the drone to which you want to connect (default : 192.168.42.1).\n\n"
          << "  --hd-resolution\n"
          << "     Enables HD 720p video instead of default 480p.\n\n"
          << "  --verbose, -v\n"
          << "      Enables verbose (drone information messages are then displayed).\n\n"
          << "  --help, -h\n"
          << "     Print help message.\n\n"
          << std::endl;
        return EXIT_SUCCESS;
      }
      else {
        std::cout << "Error : unknown parameter " << argv[i] << std::endl
          << "See " << argv[0] << " --help" << std::endl;
        return EXIT_FAILURE;
      }
    }

    vpRobotBebop2 drone(
        verbose, true, ip_address); // Create the drone with desired verbose level, settings reset, and corresponding IP

    if (drone.isRunning()) {

      drone.setVideoResolution(stream_res); // Set video resolution to 480p (default) or 720p

      drone.startStreaming(); // Start video decoding and streaming

      vpImage<vpRGBa> I(1, 1, 0);
      drone.getRGBaImage(I); // Get color image from the drone video stream

#ifdef VISP_HAVE_X11
      vpDisplayX d(I);
#elif defined(VISP_HAVE_GDI)
      vpDisplayGDI d(I);
#elif defined(HAVE_OPENCV_HIGHGUI)
      vpDisplayOpenCV d(I);
#else
      std::cout << "No image viewer is available..." << std::endl;
#endif
      vpDisplay::display(I);
      vpDisplay::flush(I);

      drone.doFlatTrim();
      drone.takeOff(true);

      vpColVector vel(4, 0.0);
      vel[3] = vpMath::rad(10);

      double delta_t = 0.040;
      double t = vpTime::measureTimeMs();

      do { // We make the drone rotate around Z axis for 10 seconds at 10 deg/s
        drone.setVelocity(vel, 1);

        drone.getRGBaImage(I);
        vpDisplay::display(I);
        vpDisplay::flush(I);

        vpTime::wait(delta_t * 1000);
      } while (vpTime::measureTimeMs() - t < 10 * 1000);

      drone.land();

    }
    else {
      std::cout << "Error : failed to setup drone control" << std::endl;
    }

    std::cout << "-- End of test --" << std::endl;
  }
  catch (const vpException &e) {
    std::cout << "Caught an exception: " << e << std::endl;
  }
#else
  (void)argc;
  (void)argv;
  std::cout << "Install Parrot ARSDK, configure and build ViSP to use this example..." << std::endl;
#endif
}
