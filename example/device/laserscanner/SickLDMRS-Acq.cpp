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
 * Sick LD-MRS laser driver.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \example SickLDMRS-Acq.cpp

  \brief Example that shows how to acquire Sick LD-MRS laser
  measurements.

  \warning For the moment, this example is only working on UNIX
  platforms since the Sick LD-MRS driver was not ported to Windows.

*/
#include <visp3/core/vpDebug.h>
#include <visp3/io/vpParseArgv.h>
#include <visp3/sensor/vpSickLDMRS.h>

#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__)))

int main()
{
  try {
    vpSickLDMRS laser;
    std::string ip = "131.254.12.119";

    laser.setIpAddress(ip);
    laser.setup();
    unsigned long int iter = 0;

    for (;;) {
      double t1 = vpTime::measureTimeMs();
      vpLaserScan laserscan[4];
      if (laser.measure(laserscan) == false)
        continue;

      iter++;
      std::cout << "iter: " << iter << " time: " << vpTime::measureTimeMs() - t1 << " ms" << std::endl;
    }
    return EXIT_SUCCESS;
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
}

#else  // #ifdef UNIX
int main()
{
  std::cout << "This example is only working on unix-like platforms \n"
            << "since the Sick LD-MRS driver was not ported to Windows." << std::endl;

  return EXIT_SUCCESS;
}
#endif // #ifdef UNIX
