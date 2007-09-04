/****************************************************************************
 *
 * $Id: ringLight.cpp,v 1.1 2007-09-04 09:32:19 fspindle Exp $
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

// List of allowed command line options
#define GETOPTARGS	"d:h"

/*!

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.

*/
void usage(char *name, char *badparam)
{
  fprintf(stdout, "\n\
Send a pulse to activate the ring light.\n\
\n\
SYNOPSIS\n\
  %s [-h]\n\
", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
\n\
  -h\n\
     Print the help.\n\n");

  if (badparam) {
    fprintf(stderr, "ERROR: \n" );
    fprintf(stderr, "\nBad parameter [%s]\n", badparam);
  }

}

/*!

  Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.

  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, char **argv)
{
  char *optarg;
  int	c;

  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg)) > 1) {

    switch (c) {
    case 'h': usage(argv[0], NULL); return false; break;

    default:
      usage(argv[0], optarg); return false; break;
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

/*!

  Send a data to the parallel port.

*/
int
main(int argc, char **argv)
{

  // Read the command line options
  if (getOptions(argc, argv) == false) {
    exit (-1);
  }
  try {

    vpRingLight light;

    printf("Activates the ring light\n");
    light.activate();
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
