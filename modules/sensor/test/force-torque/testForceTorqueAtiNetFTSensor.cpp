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
 * Test force/torque ATI sensor.
 */

/*!
  \example testForceTorqueAtiNetFTSensor.cpp
  This example shows how to retrieve data from an ATI F/T sensor connected to
  a Net F/T box.
*/

#include <iostream>

#include <visp3/gui/vpPlot.h>
#include <visp3/sensor/vpForceTorqueAtiNetFTSensor.h>

int main(int argc, char **argv)
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  // Check if vpForceTorqueAtiNetFTSensor can be used since inet_ntop() used to
  // communicate by UDP with the sensor is not supported on win XP
#ifdef VISP_HAVE_FUNC_INET_NTOP

  std::string opt_ip = "192.168.1.1";
  int opt_port = 49152;
  bool opt_no_display = false;

  for (int i = 0; i < argc; i++) {
    if (std::string(argv[i]) == "--ip")
      opt_ip = std::string(argv[i + 1]);
    else if (std::string(argv[i]) == "--port")
      opt_port = atoi(argv[i + 1]);
    else if (std::string(argv[i]) == "--no-display" || std::string(argv[i]) == "-d")
      opt_no_display = true;
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "\nUsage: " << argv[0]
        << " [--ip <Net F/T IP address (default: 192.168.1.1)>] [--port <Ethernet port (default: 49152)>]"
        << " [--no-display] [-d] [--help] [-h]\n"
        << std::endl;
      return EXIT_SUCCESS;
    }
  }

  std::cout << "Use IP  : " << opt_ip << std::endl;
  std::cout << "Use port: " << opt_port << std::endl;
  std::cout << "Disable display: " << opt_no_display << std::endl;

  vpForceTorqueAtiNetFTSensor ati_net_ft;
  ati_net_ft.init(opt_ip, opt_port);
  if (!ati_net_ft.startStreaming()) {
    std::cout << "Unable to start streaming" << std::endl;
    return EXIT_FAILURE;
  }

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
  bool bias = false;
#endif

  vpColVector ft;
  bool end = false;
  unsigned long nbacq = 0;
  double t_start = vpTime::measureTimeMs();
  while (!end) {
    double t = vpTime::measureTimeMs();

    if (ati_net_ft.waitForNewData()) {
      ft = ati_net_ft.getForceTorque();
#if defined(VISP_HAVE_DISPLAY)
      if (!opt_no_display) {
        vpColVector force = ft.extract(0, 3);
        vpColVector torque = ft.extract(3, 3);
        plotter->plot(0, nbacq, force);
        plotter->plot(1, nbacq, torque);
        vpDisplay::displayText(plotter->I, 20, 80, "Left click to quit", vpColor::red);
        if (bias) {
          vpDisplay::displayText(plotter->I, 40, 80, "Right click to unbias", vpColor::red);
        }
        else {
          vpDisplay::displayText(plotter->I, 40, 80, "Right click to bias", vpColor::red);
        }
        vpMouseButton::vpMouseButtonType button;

        if (vpDisplay::getClick(plotter->I, button, false)) {
          if (button == vpMouseButton::button1) {
            end = true;
          }
          else if (button == vpMouseButton::button3) {
            bias = !bias;
            if (bias) {
              std::cout << "Bias F/T sensor" << std::endl;
              ati_net_ft.bias();
            }
            else {
              std::cout << "Unbias F/T sensor" << std::endl;
              ati_net_ft.unbias();
            }
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
      nbacq++;
    }
    vpTime::wait(t, 2);
    std::cout << "Loop time: " << vpTime::measureTimeMs() - t << " ms" << std::endl;
  }
  ati_net_ft.stopStreaming();
  double fps = 1000. * nbacq / (vpTime::measureTimeMs() - t_start);

#if defined(VISP_HAVE_DISPLAY)
  if (plotter) {
    delete plotter;
  }
#endif

  std::cout << "Mean acquisition frequency: " << fps << " Hz" << std::endl;
  std::cout << "Test succeed" << std::endl;
#else
  (void)argc;
  (void)argv;
  std::cout << "vpForceTorqueAtiNetFTSensor is not supported on this platform" << std::endl;
#endif
  return EXIT_SUCCESS;
}
