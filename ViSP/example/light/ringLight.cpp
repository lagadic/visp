/****************************************************************************
 *
 * $Id$
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit
 *
 * This file may be distributed under the terms of the Q Public License
 * as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE included in the packaging of this file.
 *
 * Licensees holding valid ViSP Professional Edition licenses may
 * use this file in accordance with the ViSP Commercial License
 * Agreement provided with the Software.
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
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

#include <visp/vpConfig.h>
#include <visp/vpDebug.h>

#if defined VISP_HAVE_PARPORT
#include <stdlib.h>
#include <stdio.h>
#include <iostream>

#include <visp/vpRingLight.h>
#include <visp/vpParseArgv.h>
#include <visp/vpTime.h>

// List of allowed command line options
#define GETOPTARGS	"d:hn:ot:"

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
    fprintf(stderr, "ERROR: \n" );
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
  int	c;

  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg)) > 1) {

    switch (c) {
    case 'n': nsec = atoi(optarg); break;
    case 'o': on = true; break;
    case 't': nmsec = atof(optarg); break;
    case 'h': usage(argv[0], NULL, nsec, nmsec); return false; break;

    default:
      usage(argv[0], optarg, nsec, nmsec); return false; break;
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
int
main(int argc, const char **argv)
{
  bool on = false;
  int nsec = 5; // Time while the ring light is turned on
  double nmsec = 0; // Pulse duration

  // Read the command line options
  if (getOptions(argc, argv, on, nsec, nmsec) == false) {
    exit (-1);
  }
  try {

    vpRingLight light;

    if (nmsec == 0.)
      light.pulse();
    else
      light.pulse(nmsec);

    if (on) {
      printf("Turn on ring light\n");
      light.on(); // Turn the ring light on
      vpTime::wait(nsec * 1000); // Wait 5 s
      light.off(); // and then turn the ring light off
    }
    else {
      printf("Send a pulse to activate the ring light\n");
      light.pulse();
    }
  }
  catch (vpParallelPortException e) {
    switch(e.getCode()) {
    case vpParallelPortException::opening:
      printf("Can't open the parallel port to access to the ring light device\n");
      break;
    case vpParallelPortException::closing:
      printf("Can't close the parallel port\n");
      break;
    }
  }
  catch(...) {
      printf("An error occurs...\n");
  }
  return 0;
}
#else
int
main()
{
  vpTRACE("Sorry, for the moment, vpRingLight class works only on unix...");
  return 0;
}
#endif
