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
 * Test force/torque ATI sensor.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \example testComedi.cpp
  This example shows how to retrieve data from a sensor connected to a DAQ
  board. He we have 1 signe main threads that acquires physical values at 100
  Hz (10 ms) and records data in recorded-physical-data-sync.txt file.

*/
#include <iostream>
#include <sstream>

#include <visp3/gui/vpPlot.h>
#include <visp3/sensor/vpComedi.h>

int main()
{
#ifdef VISP_HAVE_COMEDI
  vpComedi comedi;
  comedi.setDevice("/dev/comedi0");
  comedi.setChannelNumbers(6); // to read a F/T tensor
  comedi.open();

#ifdef VISP_HAVE_DISPLAY
  vpPlot scope(1, 700, 700, 100, 200,
               std::string("ATI physical sensor data (") + comedi.getPhyDataUnits() + std::string(")"));
  scope.initGraph(0, comedi.getNChannel());
  for (unsigned int i = 0; i < comedi.getNChannel(); i++)
    scope.setLegend(0, i, "G" + dynamic_cast<std::ostringstream &>((std::ostringstream() << i)).str());
#endif

  std::string file("recorded-physical-data-sync.txt");
  std::ofstream f(file.c_str());

  double start_time = vpTime::measureTimeMs();
#ifdef VISP_HAVE_DISPLAY
  while (!vpDisplay::getClick(scope.I, false)) // Stop recording by a user click
#else
  std::cout << "Data recording during 20 seconds in progress..." << std::endl;
  while (vpTime::measureTimeMs() - start_time < 20000) // Stop recording after 20 seconds
#endif
  {
    double loop_time = vpTime::measureTimeMs();
    vpColVector phydata = comedi.getPhyData();
    double timestamp = loop_time - start_time;

    f << timestamp << " " << phydata.t() << std::endl;

#ifdef VISP_HAVE_DISPLAY
    scope.plot(0, timestamp, phydata);
#endif
    vpTime::wait(loop_time, 10);
  }

  f.close();

#else
  std::cout << "You should install comedi to enable this test..." << std::endl;
#endif
}
