/****************************************************************************
 *
 * $Id: parse-argv2.cpp,v 1.1 2007-05-02 16:40:13 fspindle Exp $
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
  \file parse-argv.cpp

  \brief Parsing command line arguments.
*/


/*!
  \example parse-argv.cpp

  Example of command line parsing.
*/

#include <stdio.h>
#include <sstream>
#include <iomanip>

#include <visp/vpDebug.h>
#include <visp/vpParseArgv.h>


int
main(int argc, char ** argv)
{
  using ::std::cout;
  using ::std::endl;

  int    i_val = 3;
  float  f_val = 3.14;
  double d_val = 3.1415;

  vpArgvInfo argTable[] =
    {
      {"-integer", ARGV_INT, (char*) NULL, (char *) &i_val,
	 "An integer value."},
      {"-float", ARGV_FLOAT, (char*) NULL, (char *) &f_val,
       "A float value."},
      {"-double", ARGV_DOUBLE, (char*) NULL, (char *) &d_val,
       "A double value."},
      {(char*) NULL, ARGV_END, (char*) NULL, (char*) NULL, (char*) NULL}
    } ;

  // Read the command line options
  if(vpParseArgv::parse(&argc, argv, argTable, 0)) {
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
