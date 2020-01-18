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
 * Example of ring light control.
 *
 * Author:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \example ringLight.cpp

  Shows how to activates the ring light.
*/

#include <cmath>  // std::fabs
#include <limits> // numeric_limits
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h>
#if defined VISP_HAVE_PARPORT
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include <visp3/core/vpTime.h>
#include <visp3/io/vpParseArgv.h>
#include <visp3/robot/vpRingLight.h>

// List of allowed command line options
#define GETOPTARGS "d:hn:ot:"

/*!

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param nsec : Time to wait in seconds
  \param nmsec : Pulse duration in ms

*/
void usage(const char *name, const char *badparam, int nsec, double nmsec)
{
  fprintf(stdout, "\n\
Send a pulse to activate the ring light or turn on the ring light \n\
during %d s.\n\
\n\
By default, that means without parameters, send a pulse which duration\n\
is fixed by the harware. To control the duration of the pulse, use \n\
\"-t <pulse width in ms>\" option. To turn on the light permanently, \n\
use \"-o -n <on duration in second>]\"\n			       \
\n\
SYNOPSIS\n\
  %s [-o] [-n <on duration in second>] [-t <pulse width in ms>] [-h]\n\
", nsec, name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
\n\
  -o\n\
     Turn the ring light on during %d s.\n\
     If this option is not set, send a short pulse\n\
     to activate the light.\n\
\n\
  -t %%g : <pulse width in ms>                              %g\n\
     Pulse width in milli-second.\n\
     Send a pulse which duration is fixed by this parameter.\n\
     Without this option, the pulse width is fixed by the \n\
     harware.\n\
\n\
  -n %%d : <on duration in second>                          %d\n\
     Time in second while the ring light is turned on.\n\
     This option is to make into realtion with option \"-o\".\n\
\n\
  -h\n\
     Print the help.\n\n", nsec, nmsec, nsec);

  if (badparam) {
    fprintf(stderr, "ERROR: \n");
    fprintf(stderr, "\nBad parameter [%s]\n", badparam);
  }
}

/*!

  Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param on : When true, turn on the light during nsec seconds.
  \param nsec : Time to wait in seconds.
  \param nmsec : Pulse duration in milli-seconds.

  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv, bool &on, int &nsec, double &nmsec)
{
  const char *optarg;
  int c;

  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg)) > 1) {

    switch (c) {
    case 'n':
      nsec = atoi(optarg);
      break;
    case 'o':
      on = true;
      break;
    case 't':
      nmsec = atof(optarg);
      break;
    case 'h':
      usage(argv[0], NULL, nsec, nmsec);
      return false;
      break;

    default:
      usage(argv[0], optarg, nsec, nmsec);
      return false;
      break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, nsec, nmsec);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg << std::endl << std::endl;
    return false;
  }

  return true;
}

/*!

  Send a data to the parallel port.

*/
int main(int argc, const char **argv)
{
  try {
    bool on = false;
    int nsec = 5;     // Time while the ring light is turned on
    double nmsec = 0; // Pulse duration

    // Read the command line options
    if (getOptions(argc, argv, on, nsec, nmsec) == false) {
      exit(-1);
    }

    vpRingLight light;

    // if (nmsec == 0.)
    if (std::fabs(nmsec) <= std::numeric_limits<double>::epsilon())
      light.pulse();
    else
      light.pulse(nmsec);

    if (on) {
      printf("Turn on ring light\n");
      light.on();                // Turn the ring light on
      vpTime::wait(nsec * 1000); // Wait 5 s
      light.off();               // and then turn the ring light off
    } else {
      printf("Send a pulse to activate the ring light\n");
      light.pulse();
    }
  } catch (vpParallelPortException &e) {
    switch (e.getCode()) {
    case vpParallelPortException::opening:
      printf("Can't open the parallel port to access to the ring light "
             "device\n");
      break;
    case vpParallelPortException::closing:
      printf("Can't close the parallel port\n");
      break;
    }
  } catch (...) {
    printf("An error occurs...\n");
  }
  return EXIT_SUCCESS;
}
#else
int main()
{
  std::cout << "vpRingLight class works only on unix on a the Inria Afma6 platform..." << std::endl;
  return EXIT_SUCCESS;
}
#endif
