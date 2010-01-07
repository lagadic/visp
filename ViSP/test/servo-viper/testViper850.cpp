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
 * Test for Viper850 6 dof robot.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \example testViper850.cpp

  Example of a real robot control, the Viper robot (arm robot, with 6
  degrees of freedom).
*/

#include <iostream>


#include <visp/vpConfig.h>
#include <visp/vpViper850.h>
#include <visp/vpDebug.h>


int main()
{
  try {

    std::cout << "a test for vpViper850 class..." << std::endl;

    vpViper850 viper850;
    vpCameraParameters cam;

    std::cout << "-- Default settings for Viper 850  ---" 
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

    vpColVector qmotor(6);
#if 1
    qmotor[0] = vpMath::rad(30);
    qmotor[1] = vpMath::rad(-100);
    qmotor[2] = vpMath::rad(180);
    qmotor[3] = vpMath::rad(20);
    qmotor[4] = vpMath::rad(90);
    qmotor[5] = vpMath::rad(13.37);
#else
    qmotor[0] = vpMath::rad(0);
    qmotor[1] = vpMath::rad(0-90);
    qmotor[2] = vpMath::rad(0);
    qmotor[3] = vpMath::rad(0);
    qmotor[4] = vpMath::rad(0);
    qmotor[5] = vpMath::rad(0);
#endif
    vpHomogeneousMatrix fMe;
    viper850.get_fMe(qmotor, fMe);

    vpTranslationVector t;
    fMe.extract(t);
    vpRotationMatrix R;
    fMe.extract(R);
    vpRzyzVector r;
    r.buildFrom(R);

    std::cout << "fMe:" << std::endl
	      << "\tt: " << t.t() << std::endl
	      << "\trzyz (rad): " << r.t() << std::endl
	      << "\trzyz (deg): " << vpMath::deg(r[0]) 
	      << " " << vpMath::deg(r[1]) 
	      << " " << vpMath::deg(r[2]) << std::endl;

    return 0;
  }
  catch(...) {
    vpERROR_TRACE(" Test failed");
    return 0;
  }
}

