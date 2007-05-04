/****************************************************************************
 *
 * $Id: parse-argv1.cpp,v 1.3 2007-05-04 16:06:44 fspindle Exp $
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
 * Example of  command line parsing.
 *
 * Author:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \file parse-argv1.cpp

  \brief Parsing command line arguments.
*/


/*!
  \example parse-argv1.cpp

  Example of command line parsing.
*/

#include <stdio.h>
#include <sstream>
#include <iomanip>

#include <visp/vpDebug.h>
#include <visp/vpParseArgv.h>

// List of allowed command line options
#define GETOPTARGS	"d:f:i:h"

/*!

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param i_val : An integer.
  \param f_val : A float.
  \param d_val : A double.

*/
void usage(char *name, char *badparam, int i_val, float f_val, double d_val)
{
  fprintf(stdout, "\n\
Parsing command line arguments example.\n\
\n\
SYNOPSIS\n\
  %s [-i <integer>] [-f <float>] [-d <double> [-h]\n\
", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -i <integer>                                         %d\n\
     An integer value.\n\
\n\
  -f <float>                                           %f\n\
     A float value.\n\
\n\
  -d <double>                                          %g\n\
     A double value.\n\
\n\
  -h\n\
     Print the help.\n\n",
	  i_val, f_val, d_val);

  if (badparam) {
    fprintf(stderr, "ERROR: \n" );
    fprintf(stderr, "\nBad parameter [%s]\n", badparam);
  }
}

/*!

  Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param i_val : An integer.
  \param f_val : A float.
  \param d_val : A double.
  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, char **argv, int &i_val, float &f_val, double &d_val)
{
  char *optarg;
  int	c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg)) > 1) {

    switch (c) {
    case 'd': d_val = atof(optarg); break;
    case 'f': f_val = (float) atof(optarg); break;
    case 'i': i_val = atoi(optarg); break;
    case 'h': usage(argv[0], NULL, i_val, f_val, d_val); return false; break;

    default:
      usage(argv[0], optarg, i_val, f_val, d_val); return false; break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, i_val, f_val, d_val);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg << std::endl << std::endl;
    return false;
  }

  return true;
}

int
main(int argc, char ** argv)
{
  using ::std::cout;
  using ::std::endl;

  int    i_val = 3;
  float  f_val = 3.14f;
  double d_val = 3.1415;

    // Read the command line options
  if (getOptions(argc, argv, i_val, f_val, d_val) == false) {
    return (-1);
  }

  cout << "Your parameters: " << endl;
  cout << "  Integer value: " << i_val << endl;
  cout << "  Float   value: " << f_val << endl;
  cout << "  Double  value: " << d_val << endl << endl;
  cout << "Call  " << argv[0]
       << " -h to see how to change these parameters." << endl;

  return 0;
}



/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
