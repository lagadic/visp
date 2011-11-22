/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2011 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
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
#include <visp/vpConfig.h>
#if defined UNIX
#  include <unistd.h>
#elif defined WIN32
#  include <windows.h>
#  include <mmsystem.h>
#  include <winbase.h>
#endif
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <time.h>
#include <visp/vpTime.h>
#include <visp/vpParseArgv.h>



// List of allowed command line options
#define GETOPTARGS	"h"

/*!

  Print the program options.

*/
void usage(const char *name, const char *badparam)
{
  fprintf(stdout, "\n\
Time management.\n\
\n\
SYNOPSIS\n\
  %s [-h]\n", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -h\n\
     Print the help.\n");

  if (badparam)
    fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
}
/*!

  Set the program options.

  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv)
{
  const char *optarg;
  int	c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg)) > 1) {

    switch (c) {
    case 'h': usage(argv[0], NULL); return false; break;

    default:
      usage(argv[0], optarg);
      return false; break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg << std::endl << std::endl;
    return false;
  }

  return true;
}


int
main(int argc, const char ** argv)
{
  // Read the command line options
  if (getOptions(argc, argv) == false) {
    exit (-1);
  }

  double v = 0;

  double t0 = vpTime::measureTimeMs();
  for (int i =0 ; i < 100000; i ++)
    for (int j =0 ; j < 100; j ++)
      v = i * 2 / 3. + j;
  std::cout << "Computed dummy value: " << v << std::endl;

  double t1 = vpTime::measureTimeMs();
  vpTime::wait(t1, 40);

  double t2 = vpTime::measureTimeMs();

  // Sleep 10ms
#if defined UNIX
  usleep(10*1000);
#elif defined WIN32
  Sleep(10);
#endif

  double t3 = vpTime::measureTimeMs();

  // Sleep 2ms
#if defined UNIX
  usleep(2*1000);
#elif defined WIN32
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
}
