/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
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
 * Hello world example.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#include <iostream>
#include <limits>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpRotationMatrix.h>
#include <visp3/core/vpThetaUVector.h>

int main()
{
  try {
    vpThetaUVector tu;

    // Construct a rotation matrix from the theta U angles
    vpRotationMatrix R(vpMath::rad(0.), vpMath::rad(180) + 100 * std::numeric_limits<double>::epsilon(), 0.);

    // Extract the theta U angles from a rotation matrix
    tu.buildFrom(R);

    // Since the rotation vector is 3 values column vector, the
    // transpose operation produce a row vector.
    vpRowVector tu_t = tu.t();

    // Print the transpose row vector
    std::cout << tu_t << std::endl;
    return EXIT_SUCCESS;
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
}
