/****************************************************************************
 *
 * $Id: test_robust.cpp,v 1.3 2006-07-10 16:44:45 fspindle Exp $
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
 * Test some vpMath functionalities.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \example test_robust.cpp

  Test some vpMath functionalities.
*/

#include<iostream>
#include<visp/vpRobust.h>
#include <fstream>
#include <visp/vpIoTools.h>
#include <visp/vpParseArgv.h>

// List of allowed command line options
#define GETOPTARGS	"ho:"

/*!

  Print the program options.

*/
void usage(char *name, char *badparam, string ofilename)
{
  fprintf(stdout, "\n\
Test some vpMath functionalities. Compute weights and print\n\
them in an output file.\n\
\n\
Using gnuplot the content of the output file can be printed by:\n\
set style data line\n\
set ylabel \"weight\"\n\
set yr [0:1.19]\n\
set xlabel \"Normalized residuals\"\n\
plot '%s' title \"Tukey Estimator\" lw 2, 1 title \"Least-Squares\" lw 2\n\
\n\
\n\
SYNOPSIS\n\
  %s [-o <output filename>] [-h]\n", ofilename.c_str(), name);

  fprintf(stdout, "\n\
OPTIONS:                                              Default\n\
  -o <output filename>                                %s\n\
     Name and path of the file containing computed \n\
     weights.\n\
\n\
  -h\n\
     Print the help.\n",
	  ofilename.c_str());

}
/*!

  Set the program options.

  \param ofilename : Output filename.
  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, char **argv, string &ofilename)
{
  char *optarg;
  int	c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg)) > 1) {

    switch (c) {
    case 'o': ofilename = optarg; break;
    case 'h': usage(argv[0], NULL, ofilename); return false; break;

    default:
      usage(argv[0], optarg, ofilename); 
      return false; break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, ofilename); 
    cerr << "ERROR: " << endl;
    cerr << "  Bad argument " << optarg << endl << endl;
    return false;
  }

  return true;
}



int
main(int argc, char ** argv)
{
  string ofilename;
  string username;

  // Set the default output filename
#ifdef UNIX
  ofilename = "/tmp";
#elif WIN32
  ofilename = "C:/temp";
#endif

  // Get the user login name
  vpIoTools::getUserName(username);

  // Append to the output filename string, the login name of the user
  ofilename = ofilename + "/" + username;

  // Test if the output path exist. If no try to create it
  if (vpIoTools::checkDirectory(ofilename) == false) {
    try {
      // Create the dirname
      vpIoTools::makeDirectory(ofilename);
    }
    catch (...) {
      usage(argv[0], NULL, ofilename);
      cerr << endl 
	   << "ERROR:" << endl;
      cerr << "  Cannot create " << ofilename << endl;
      cerr << "  Check your -o " << ofilename << " option " << endl;
      exit(-1);
    }
  }

  // Append to the output filename string, the name of the file
  ofilename = ofilename + "/w.dat";

  // Read the command line options
  if (getOptions(argc, argv, ofilename) == false) {
    exit (-1);
  }

  double sig = 1 ;

  double w ;
  ofstream f;
  cout << "Create file: " << ofilename << endl;
  f.open(ofilename.c_str());
  if (f == NULL) {
    usage(argv[0], NULL, ofilename);
    cerr << endl 
	 << "ERROR:" << endl;
    cerr << "  Cannot create the file: " << ofilename << endl;
    cerr << "  Check your -o " << ofilename << " option " << endl;
    exit(-1);

  }
  double x = -10 ;
  while (x<10)
  {
    if (fabs(x/sig)<=(4.6851))
    {
      w = vpMath::sqr(1-vpMath::sqr(x/(sig*4.6851)));
    }
    else
    {
      w = 0;
    }
    f << x <<"  "<<w <<endl ;
    x+= 0.01 ;
  }
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
