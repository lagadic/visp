/****************************************************************************
 *
 * $Id: ringLight.cpp,v 1.3 2007-10-19 08:47:05 fspindle Exp $
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

#if defined UNIX
#include <stdio.h>
#include <iostream>

#include <visp/vpRingLight.h>
#include <visp/vpParseArgv.h>
#include <visp/vpTime.h>

// List of allowed command line options
#define GETOPTARGS	"d:hn:o"

/*!

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param nsec : Time to wait in seconds

*/
void usage(char *name, char *badparam, int nsec)
{
  fprintf(stdout, "\n\
Send a pulse to activate the ring light or turn onthe  ring light \n\
during %d s.\n\
\n\
SYNOPSIS\n\
  %s [-o] [-n nsecond] [-h]\n\
", nsec, name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
\n\
  -o\n\
     Turn the ring light on during %d s.\n\
     If this option is not set, send a short pulse\n\
     to activate the light.\n\
\n\
  -n %%d                                                  %d\n\
     Time in second while the ring light is turned on.\n\
     This option is to make into realtion with option \"-o\".\n\
\n\
  -h\n\
     Print the help.\n\n", nsec, nsec);

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
  \param nsec : Time to wait in seconds

  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, char **argv, bool &on, int &nsec)
{
  char *optarg;
  int	c;

  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg)) > 1) {

    switch (c) {
    case 'o': on = true; break;
    case 'n': nsec = atoi(optarg); break;
    case 'h': usage(argv[0], NULL, nsec); return false; break;

    default:
      usage(argv[0], optarg, nsec); return false; break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, nsec);
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
main(int argc, char **argv)
{
  bool on = false;
  int nsec = 5; // Time while the ring light is turned on

  // Read the command line options
  if (getOptions(argc, argv, on, nsec) == false) {
    exit (-1);
  }
  try {

    vpRingLight light;

    light.pulse();

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
