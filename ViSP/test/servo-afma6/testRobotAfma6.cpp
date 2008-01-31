/****************************************************************************
 *
 * $Id: testRobotAfma6.cpp,v 1.16 2008-01-31 15:01:40 asaunier Exp $
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
  \example testRobotAfma6.cpp

  Example of a real robot control, the Afma6 robot (cartesian robot, with 6
  degrees of freedom).
*/

#include <iostream>


#include <visp/vpConfig.h>
#include <visp/vpRobotAfma6.h>
#include <visp/vpDebug.h>

#ifdef VISP_HAVE_AFMA6
#ifdef VISP_HAVE_GSL
int gsl_warnings_off;
#endif

int main()
{
  try {

    std::cout << "a test..." << std::endl;

    vpAfma6 afma6;
    vpCTRACE << afma6;

    vpRobotAfma6 robotAfma6;

    vpCTRACE << "-- set camera Dragonfly2 without distortion ---" << std::endl;
    robotAfma6.init( vpAfma6::CAMERA_DRAGONFLY2_8MM);

    vpCTRACE << robotAfma6;

    vpCTRACE << "-- set camera Dragonfly2 with distortion ------" << std::endl;
    robotAfma6.init( vpAfma6::CAMERA_DRAGONFLY2_8MM,
                     vpCameraParameters::perspectiveProjWithDistortion);
    vpCTRACE << robotAfma6;
    return 0;
  }
  catch(...) {
    vpERROR_TRACE(" Test failed");
    return 0;
  }

}
#else
int main()
{
  std::cout << "a test..." << std::endl;
  return 0; 
}

#endif
