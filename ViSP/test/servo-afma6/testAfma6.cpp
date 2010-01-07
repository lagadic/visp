/****************************************************************************
 *
 * $Id$
 *
 * Copyright (C) 1998-2010 Inria. All rights reserved.
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
  \example testAfma6.cpp

  Example of a real robot control, the Afma6 robot (cartesian robot, with 6
  degrees of freedom).
*/

#include <iostream>


#include <visp/vpConfig.h>
#include <visp/vpAfma6.h>
#include <visp/vpDebug.h>
#include <visp/vpCameraParameters.h>


int main()
{
  try {

    std::cout << "a test for vpAfma6 class..." << std::endl;

    vpAfma6 afma6;
    vpCameraParameters cam;

    std::cout << "-- Default settings for Afma6  ---" 
	      << std::endl;
    std::cout << afma6 << std::endl;
    afma6.getCameraParameters(cam, 640, 480);
    std::cout << cam << std::endl;

    std::cout << "-- Settings associated to the CCMOP tool without distortion ---" 
	      << std::endl;
    afma6.init( vpAfma6::TOOL_CCMOP);

    std::cout << afma6 << std::endl;
    afma6.getCameraParameters(cam, 640, 480);
    std::cout << cam << std::endl;

    std::cout << "-- Settings associated to the CCMOP tool with distortion ------" 
	      << std::endl;
    afma6.init( vpAfma6::TOOL_CCMOP,
		vpCameraParameters::perspectiveProjWithDistortion);
    std::cout << afma6 << std::endl;
    afma6.getCameraParameters(cam, 640, 480);
    std::cout << cam << std::endl;

    std::cout << "-- Settings associated to the gripper tool without distortion ---" 
	      << std::endl;
    afma6.init( vpAfma6::TOOL_GRIPPER);

    std::cout << afma6 << std::endl;
    afma6.getCameraParameters(cam, 640, 480);
    std::cout << cam << std::endl;

    std::cout << "-- Settings associated to the gripper tool with distortion ------" 
	      << std::endl;
    afma6.init( vpAfma6::TOOL_GRIPPER,
		vpCameraParameters::perspectiveProjWithDistortion);
    std::cout << afma6 << std::endl;
    afma6.getCameraParameters(cam, 640, 480);
    std::cout << cam << std::endl;

    return 0;
  }
  catch(...) {
    vpERROR_TRACE(" Test failed");
    return 0;
  }

}

