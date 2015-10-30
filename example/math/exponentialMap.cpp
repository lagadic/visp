/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2015 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * Test some vpColVector functionalities.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

/*!
  \example exponentialMap.cpp

  Test some vpExponentialMap functionalities.
*/


#include <visp3/core/vpTranslationVector.h>
#include <visp3/core/vpRotationVector.h>
#include <visp3/core/vpThetaUVector.h>
#include <visp3/core/vpRxyzVector.h>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpExponentialMap.h>


int
main()
{
  try {
    vpTranslationVector t;
    t[0] = 0.1;  // t_x in m/s
    t[1] = 0.2f; // t_y in m/s
    t[2] = 0.f;  // t_z in m/s

    vpRxyzVector rxyz;
    rxyz[0] = vpMath::rad(0.f);  // r_x in rad/s
    rxyz[1] = vpMath::rad(0.f);  // r_y in rad/s
    rxyz[2] = vpMath::rad(90.f); // r_z in rad/s

    // Build a ThetaU rotation vector from a Rxyz vector
    vpThetaUVector tu;
    tu.buildFrom(rxyz);

    vpColVector v(6); // Velocity vector [t, thetaU]^t

    v[0] = t[0]; // t_x
    v[1] = t[1]; // t_y
    v[2] = t[2]; // t_z
    v[3] = tu[0]; // ThetaU_x
    v[4] = tu[1]; // ThetaU_y
    v[5] = tu[2]; // ThetaU_z

    std::cout << "Considered velocity : \n" << v << std::endl;

    vpHomogeneousMatrix M;

    // Compute the displacement from the velocity applied during 1 second
    M = vpExponentialMap::direct(v);

    {
      // Extract translation from homogenous matrix
      vpTranslationVector dt; // translation displacement
      M.extract(dt);

      // Extract rotation from homogenous matrix
      vpRotationMatrix R;
      M.extract(R);
      vpRxyzVector drxyz(R); // rotational displacement

      std::cout << "Displacement if velocity is applied during 1 s : \n"
                << dt << " " << drxyz << std::endl;
    }

    // Compute the displacement from the velocity applied during 2 seconds
    M = vpExponentialMap::direct(v, 2.f);

    {
      // Extract translation from homogenous matrix
      vpTranslationVector dt; // translation displacement
      M.extract(dt);

      // Extract rotation from homogenous matrix
      vpRotationMatrix R;
      M.extract(R);
      vpRxyzVector drxyz(R); // rotational displacement

      std::cout << "Displacement if velocity is applied during 2 s : \n"
                << dt << " " << drxyz << std::endl;
    }

    // Compute the velocity from the displacement observed during 2 seconds
    v = vpExponentialMap::inverse(M, 2.f);

    std::cout << "Velocity from displacement observed during 2 s: \n"
              << v << std::endl;
    return 0;
  }
  catch(vpException e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return 1;
  }
}
