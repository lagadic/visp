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
 * Time management.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!

  \example testTime.cpp

  \brief Time management.

*/
#include <visp3/core/vpConfig.h>
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
#include <unistd.h>
#elif defined(_WIN32)
//#include <mmsystem.h>
//#include <winbase.h>
#include <windows.h>
#endif
#include <cmath>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <visp3/core/vpTime.h>

int main()
{
#if !defined(WINRT)
  try {
    double v = 0;

    double t0 = vpTime::measureTimeMs();
    for (int i = 0; i < 100000; i++)
      for (int j = 0; j < 100; j++)
        v = i * 2 / 3. + j;
    std::cout << "Computed dummy value: " << v << std::endl;

    double t1 = vpTime::measureTimeMs();
    vpTime::wait(t1, 40);

    double t2 = vpTime::measureTimeMs();

// Sleep 10ms
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
    usleep(10 * 1000);
#elif defined(_WIN32)
    Sleep(10);
#endif

    double t3 = vpTime::measureTimeMs();

// Sleep 2ms
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
    usleep(2 * 1000);
#elif defined(_WIN32)
    Sleep(2);
#endif
    double t4 = vpTime::measureTimeMs();

    vpTime::wait(t4, 19);

    double t5 = vpTime::measureTimeMs();

    vpTime::wait(5);

    double t6 = vpTime::measureTimeMs();

    vpTime::wait(21);

    double t7 = vpTime::measureTimeMs();

    vpTime::wait(2);

    double t8 = vpTime::measureTimeMs();

    std::cout << "t1-t0: computation time: " << t1 - t0 << std::endl;

    std::cout << "t2-t1: wait(t1, 40 ms): " << t2 - t1 << std::endl;
    std::cout << "t3-t2: sleep(10 ms): " << t3 - t2 << std::endl;
    std::cout << "t4-t3: sleep(2 ms): " << t4 - t3 << std::endl;
    std::cout << "t5-t4: wait(t, 19 ms): " << t5 - t4 << std::endl;
    std::cout << "t6-t5: wait(5 ms): " << t6 - t5 << std::endl;
    std::cout << "t7-t6: wait(21 ms): " << t7 - t6 << std::endl;
    std::cout << "t8-t7: wait(2 ms): " << t8 - t7 << std::endl;

    return 0;
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return 1;
  }
#else
  std::cout << "vpTime is not implemented on Universal Windows Platform" << std::endl;
#endif
}
