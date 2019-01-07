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
 * Tests transformation within various representations of rotation.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

/*!
  \file testDisplacement.cpp
  \brief Tests transformation within various representations of rotation
*/

/*!
  \example testDisplacement.cpp
*/

#include <stdio.h>
#include <stdlib.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpRotationMatrix.h>
#include <visp3/core/vpThetaUVector.h>
#include <visp3/vision/vpHomography.h>

bool test(const std::string &s, const vpHomography &H, const std::vector<double> &bench)
{
  static unsigned int cpt = 0;
  std::cout << "** Test " << ++cpt << std::endl;
  std::cout << s << "(" << H.getRows() << "," << H.getCols() << ") = \n[" << H << "]" << std::endl;
  if (bench.size() != H.size()) {
    std::cout << "Test fails: bad size wrt bench" << std::endl;
    return false;
  }
  for (unsigned int i = 0; i < H.size(); i++) {
    if (std::fabs(H.data[i] - bench[i]) > std::fabs(H.data[i]) * std::numeric_limits<double>::epsilon()) {
      std::cout << "Test fails: bad content" << std::endl;
      return false;
    }
  }

  return true;
}

int main()
{
  try {
    {
      vpHomography H;
      H.eye();
      std::vector<double> bench(9, 0);
      bench[0] = bench[4] = bench[8] = 1.;
      int err = 1;
      if (test("H", H, bench) == false)
        return err;
      if (test("H", H / H[2][2], bench) == false)
        return err;
    }
    {
      vpThetaUVector tu(vpMath::rad(90), vpMath::rad(120), vpMath::rad(45));

      std::cout << "Initialization " << std::endl;
      // std::cout << tu << std::endl ;

      std::cout << "From vpThetaUVector to vpRotationMatrix " << std::endl;
      vpRotationMatrix R(tu);

      // pure rotation
      vpHomogeneousMatrix M;
      M.insert(R);

      std::cout << "M" << std::endl << M << std::endl;
      vpPlane p(0, 0, 1, 1);

      vpHomography H(M, p);

      std::cout << "H" << std::endl << H << std::endl;

      vpColVector n;
      vpTranslationVector T;

      H.computeDisplacement(R, T, n);

      std::cout << "R" << std::endl << R;
      std::cout << "T" << std::endl << T.t() << std::endl;
      std::cout << "n" << std::endl << n.t() << std::endl;
    }
    std::cout << "------------------------------------------------------" << std::endl;

    {
      vpThetaUVector tu(vpMath::rad(90), vpMath::rad(120), vpMath::rad(45));

      std::cout << "Initialization " << std::endl;
      // std::cout << tu << std::endl ;

      std::cout << "From vpThetaUVector to vpRotationMatrix " << std::endl;
      vpRotationMatrix R(tu);

      // pure rotation
      vpHomogeneousMatrix M;
      M.insert(R);

      M[0][3] = 0.21;
      M[1][3] = 0.31;
      M[2][3] = 0.5;

      std::cout << "M" << std::endl << M << std::endl;
      vpPlane p(0, 0, 1, 1);

      vpHomography H(M, p);

      std::cout << "H" << std::endl << H << std::endl;

      vpColVector n;
      vpTranslationVector T;

      H.computeDisplacement(R, T, n);

      std::cout << "R" << std::endl << R;
      std::cout << "T" << std::endl << T.t() << std::endl;
      std::cout << "n" << std::endl << n.t() << std::endl;
    }

    std::cout << "------------------------------------------------------" << std::endl;
    {
      vpThetaUVector tu(vpMath::rad(-190), vpMath::rad(12), vpMath::rad(-45));

      vpRotationMatrix R(tu);

      // pure rotation
      vpHomogeneousMatrix M;
      M.insert(R);

      M[0][3] = 0.21;
      M[1][3] = -0.31;
      M[2][3] = 0.5;

      std::cout << "M" << std::endl << M << std::endl;
      vpPlane p(0.4, -0.5, 0.5, 1);

      vpHomography H(M, p);

      std::cout << "H" << std::endl << H << std::endl;

      vpColVector n;
      vpTranslationVector T;
      H.computeDisplacement(R, T, n);

      std::cout << "R" << std::endl << R;
      std::cout << "T" << std::endl << T.t() << std::endl;
      std::cout << "n" << std::endl << n.t() << std::endl;

      vpPlane p1(n[0], n[1], n[2], 1.0);
      H.buildFrom(R, T, p1);
      std::cout << "H" << std::endl << H << std::endl;
    }
    std::cout << "All tests succeed" << std::endl;
    return 0;
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return 1;
  }
}
