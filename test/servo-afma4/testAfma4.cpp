/****************************************************************************
 *
 * $Id: testAfma4.cpp,v 1.1 2008-12-15 17:19:22 fspindle Exp $
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
 * Test for Afma 6 dof robot.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \example testAfma4.cpp

  Example of a real robot control, the Afma4 robot (cartesian robot, with 6
  degrees of freedom).
*/

#include <iostream>


#include <visp/vpConfig.h>
#include <visp/vpAfma4.h>
#include <visp/vpDebug.h>
#include <visp/vpCameraParameters.h>


int main()
{
  try {

    std::cout << "a test for vpAfma4 class..." << std::endl;

    vpAfma4 afma4;

    std::cout << afma4 << std::endl;

    return 0;
  }
  catch(...) {
    vpERROR_TRACE(" Test failed");
    return 0;
  }

}

