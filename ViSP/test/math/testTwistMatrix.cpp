/****************************************************************************
 *
 * $Id: testTwistMatrix.cpp,v 1.1 2007-01-26 16:27:33 asaunier Exp $
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
 * Tests some vpMatrix functionalities.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \example testTwistMatrix.cpp

  \brief Test some vpMatrix functionalities.
*/



#include <visp/vpConfig.h>
#include <visp/vpDebug.h>


#include <visp/vpMath.h>
#include <visp/vpMatrix.h>
#include <visp/vpColVector.h>
#include <visp/vpRxyzVector.h>
#include <visp/vpTranslationVector.h>
#include <visp/vpRotationMatrix.h>
#include <visp/vpTwistMatrix.h>
#include <visp/vpParseArgv.h>

// List of allowed command line options
#define GETOPTARGS	"h"

/*!

  Print the program options.

*/
void usage(char *name, char *badparam)
{
  fprintf(stdout, "\n\
Tests some vpMatrix functionalities.\n\
\n\
SYNOPSIS\n\
  %s [-h]\n", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -h\n\
     Print the help.\n");

}
/*!

  Set the program options.

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
      usage(argv[0], optarg); 
      return false; break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL); 
    cerr << "ERROR: " << endl;
    cerr << "  Bad argument " << optarg << endl << endl;
    return false;
  }

  return true;
}


int
main(int argc, char ** argv)
{
  // Read the command line options
  if (getOptions(argc, argv) == false) {
    exit (-1);
  }

  vpTRACE("--------------------------");
  vpTRACE("--- TEST vpTwistMatrix ---");
  vpTRACE("--------------------------");

  // Set the translation
  vpTranslationVector cte;
  cte[0] = 1.;
  cte[1] = 0.5;
  cte[2] = -1.;

  // Set the rotation
  vpRxyzVector cre;
  cre[0] =  M_PI/2.;
  cre[1] = -M_PI/2.;
  cre[2] = -M_PI/4.;

  // Build rotation matrix
  vpRotationMatrix cRe(cre);

  // Build the twist matrix
  vpTwistMatrix cVe(cte, cRe);

  vpTRACE("cVe twist matrix:");
  cVe.print (std::cout, 6);


  // Set a speed skew
  vpColVector ev(6);

  ev[0] = 1.;
  ev[1] = 0.1;
  ev[2] = -0.5;
  ev[3] = M_PI/180.;
  ev[4] = M_PI/18.;
  ev[5] = M_PI/10.;

  vpTRACE("ev colvector:");
  ev.print (std::cout, 6);

  // Set a speed skew
  vpColVector cv;

  cv = cVe * ev;

  vpTRACE("cv = cVe * ev:");
  cv.print (std::cout, 6);

}
