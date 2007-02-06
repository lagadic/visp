/****************************************************************************
 *
 * $Id: myexample.cpp,v 1.1.1.1 2007-02-06 10:47:23 fspindle Exp $
 *
 * Copyright (C) 2007 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP library.
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Author:
 * Fabien Spindler
 *
 *****************************************************************************/


/*!
  \example myexample.cpp

  An example of program using the local library and the ViSP-2 library.
*/

#include <iostream>

#include <visp/vpConfig.h>
#include <visp/vpDebug.h>
#include <visp/vpMatrix.h>
#include <visp/vpColVector.h>
#include <visp/vpDisplayX.h>
#include <visp/vpRobotAfma4.h>

#include <example/exExample.h>

using namespace std;

int main()
{
  vpTRACE("An example program...");

#ifdef VISP_HAVE_AFMA4
  vpRobotAfma4 robot;
#endif
  exExample myExample;
  vpDisplayX display;

  myExample.setValue(100);

  vpCTRACE << "Value: " << myExample.getValue() << endl;

  vpMatrix M(3,3);
  vpColVector v(3);
  M = 1.0;
  v = 2.0;
  vpMatrix R;

  R = M * v;

  vpTRACE("Matrix R:");
  R.print(std::cout, 4);


  return 0;
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
