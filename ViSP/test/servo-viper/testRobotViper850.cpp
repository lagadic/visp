/****************************************************************************
 *
 * $Id$
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
  \example testRobotViper850.cpp

  Example of a real robot control, the Viper850 robot (arm, with 6
  degrees of freedom).
*/

#include <iostream>


#include <visp/vpConfig.h>
#include <visp/vpRobotViper850.h>
#include <visp/vpDebug.h>
#include <visp/vpCameraParameters.h>

#ifdef VISP_HAVE_VIPER850

int main()
{
  try {

    std::cout << "a test for vpRobotViper850 class..." << std::endl;

    vpRobotViper850 viper850;
    vpCameraParameters cam;

    std::cout << "-- Default settings for Viper850  ---" 
	      << std::endl;
    std::cout << viper850 << std::endl;
    viper850.getCameraParameters(cam, 640, 480);
    std::cout << cam << std::endl;

    std::cout << "-- Settings associated to the Marlin F033C camera without distortion ---" 
	      << std::endl;
    viper850.init( vpViper850::TOOL_MARLIN_F033C_CAMERA);

    std::cout << viper850 << std::endl;
    viper850.getCameraParameters(cam, 640, 480);
    std::cout << cam << std::endl;

    std::cout << "-- Settings associated to the Marlin F033C camera with distortion ------" 
	      << std::endl;
    viper850.init( vpViper850::TOOL_MARLIN_F033C_CAMERA,
		vpCameraParameters::perspectiveProjWithDistortion);
    std::cout << viper850 << std::endl;
    viper850.getCameraParameters(cam, 640, 480);
    std::cout << cam << std::endl;


    std::cout << "-- Current joint position:" << std::endl;
    vpColVector q;
    viper850.getPosition(vpRobot::ARTICULAR_FRAME, q);
    std::cout << "  " << q.t() << std::endl;

    std::cout << "-- Current fMe:" << std::endl;

    vpHomogeneousMatrix fMe;
    viper850.get_fMe(q, fMe);
    vpTranslationVector t;
    fMe.extract(t);
    vpRotationMatrix R;
    fMe.extract(R);
    vpRzyzVector rzyz;
    rzyz.buildFrom(R);

    std::cout << "fMe:" << std::endl
	      << "\tt: " << t.t() << std::endl
	      << "\trzyz (deg): " << vpMath::deg(rzyz[0]) 
	      << " " << vpMath::deg(rzyz[1]) 
	      << " " << vpMath::deg(rzyz[2]) << std::endl;

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
  std::cout << "The real Viper850 robot controller is not available." << std::endl;
  return 0; 
}

#endif
