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
 * Test force/torque IIT sensor.
 */

/*!
  \example testForceTorqueIitSensor.cpp
  This example shows how to retrieve data from an IIT F/T sensor connected to
  a computer by Ethernet.
*/

#include <iostream>

#include <visp3/gui/vpPlot.h>
#include <visp3/sensor/vpForceTorqueIitSensor.h>

int main(int argc, char **argv)
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
#ifdef VISP_HAVE_FT_IIT_SDK
  bool opt_no_display = false;
  bool opt_filtered = false;

  for (int i = 0; i < argc; i++) {
    if (std::string(argv[i]) == "--no-display" || std::string(argv[i]) == "-d")
      opt_no_display = true;
    if (std::string(argv[i]) == "--filtered" || std::string(argv[i]) == "-f")
      opt_filtered = true;
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "\nUsage: " << argv[0] << " [--no-display] [--filtered] [--help] [-d] [-f] [-h]\n" << std::endl;
      return 0;
    }
  }

  std::cout << "Disable display: " << opt_no_display << std::endl;

  vpForceTorqueIitSensor iit_ft;

#if defined(VISP_HAVE_DISPLAY)
  vpPlot *plotter = nullptr;
  if (!opt_no_display) {
    plotter = new vpPlot(2, 700, 700, 100, 200, "Curves...");
    plotter->initGraph(0, 3);
    plotter->setTitle(0, "Force measurements");
    plotter->setLegend(0, 0, "Fx");
    plotter->setLegend(0, 1, "Fy");
    plotter->setLegend(0, 2, "Fz");
    plotter->initGraph(1, 3);
    plotter->setTitle(1, "Torque measurements");
    plotter->setLegend(1, 0, "Tx");
    plotter->setLegend(1, 1, "Ty");
    plotter->setLegend(1, 2, "Tz");
  }
#endif

  if (!iit_ft.connected()) {
    std::cout << "Unable to connect to IIT force-torque sensor" << std::endl;
    return EXIT_SUCCESS;
  }

  iit_ft.bias();
  iit_ft.startStreaming();

  vpColVector ft;
  bool end = false;
  unsigned long nbacq = 0;
  double t_start = vpTime::measureTimeMs();
  while (!end) {
    double t = vpTime::measureTimeMs();

    ft = iit_ft.getForceTorque(opt_filtered);
#if defined(VISP_HAVE_DISPLAY)
    if (!opt_no_display) {
      vpColVector force = ft.extract(0, 3);
      vpColVector torque = ft.extract(3, 3);
      plotter->plot(0, nbacq, force);
      plotter->plot(1, nbacq, torque);
      vpDisplay::displayText(plotter->I, 20, 80, "Left click to quit", vpColor::red);
      vpMouseButton::vpMouseButtonType button;

      if (vpDisplay::getClick(plotter->I, button, false)) {
        if (button == vpMouseButton::button1) {
          end = true;
        }
      }
      vpDisplay::flush(plotter->I);
    }
    else {
      std::cout << "F/T: " << ft.t() << std::endl;
      if (nbacq > 30) {
        end = true;
      }
    }
#else
    std::cout << "F/T: " << ft.t() << std::endl;
    if (nbacq > 30) {
      end = true;
    }
#endif
    vpTime::wait(t, 1);
    nbacq++;
  }

  iit_ft.stopStreaming();
  double fps = 1000. * nbacq / (vpTime::measureTimeMs() - t_start);
  std::cout << "Mean acquisition frequency: " << fps << " Hz" << std::endl;

#if defined(VISP_HAVE_DISPLAY)
  if (plotter) {
    delete plotter;
  }
#endif

  std::cout << "Test succeed" << std::endl;

#else
  (void)argc;
  (void)argv;
  std::cout << "ViSP is not build with IIT force-torque SDK support" << std::endl;
#endif
  return EXIT_SUCCESS;
}
