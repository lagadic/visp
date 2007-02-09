/****************************************************************************
 *
 * $Id: myexample.cpp,v 1.1.1.1 2007-02-09 17:41:14 fspindle Exp $
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
 * Author:
 * Fabien Spindler
 *
 *****************************************************************************/


/*!
  \example myexample.cpp

  An example of program using the local library and the ViSP-2 library.
*/

#include <iostream>

// ViSP header
#include <visp/vpConfig.h>
#include <visp/vpDebug.h>
#include <visp/vpMatrix.h>
#include <visp/vpColVector.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayD3D.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpRobotAfma4.h>

// Project header
#include <example/exExample.h>

using namespace std;

int main()
{
  vpTRACE("An example program...");

  // Use project library functionnalities
  exExample myExample;
  myExample.setValue(100);
  vpCTRACE << "Value: " << myExample.getValue() << endl;

  // Use ViSP functions
#ifdef VISP_HAVE_AFMA4
  vpRobotAfma4 robot;
#endif

#if defined(VISP_HAVE_X11)
  vpDisplayX display;
#elif defined(VISP_HAVE_GTK)
  vpDisplayGTK display;
#elif defined(VISP_HAVE_D3D9)
  vpDisplayD3D display;
#elif defined(VISP_HAVE_GDI)
  vpDisplayGDI display;
#endif

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
