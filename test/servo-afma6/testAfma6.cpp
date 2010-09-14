/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2010 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
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

