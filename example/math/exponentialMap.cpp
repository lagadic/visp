/****************************************************************************
 *
 * $Id: exponentialMap.cpp,v 1.2 2007-05-10 14:55:47 fspindle Exp $
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


#include <visp/vpTranslationVector.h>
#include <visp/vpRotationVector.h>
#include <visp/vpThetaUVector.h>
#include <visp/vpRxyzVector.h>
#include <visp/vpColVector.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpExponentialMap.h>


int
main()
{

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

}
