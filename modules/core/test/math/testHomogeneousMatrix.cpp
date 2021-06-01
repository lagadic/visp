/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2021 by Inria. All rights reserved.
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
 * Test some vpHomogeneousMatrix functionalities.
 *
 *****************************************************************************/

/*!
  Test some vpHomogeneousMatrix functionalities.
 */
#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_CATCH2
#include <visp3/core/vpHomogeneousMatrix.h>

#define CATCH_CONFIG_RUNNER
#include <catch.hpp>

TEST_CASE("vpHomogeneousMatrix re-orthogonalize rotation matrix", "[vpHomogeneousMatrix]") {
  CHECK_NOTHROW([](){
    vpHomogeneousMatrix M {
      0.9835, -0.0581,  0.1716, 0.0072,
      -0.0489, -0.9972, -0.0571, 0.0352,
      0.1744,  0.0478, -0.9835, 0.9470
    };
  }());

  CHECK_NOTHROW([](){
    vpHomogeneousMatrix M {
      0.9835, -0.0581,  0.1716, 0.0072,
      -0.0937, -0.9738,  0.2072, 0.0481,
      0.1551, -0.2199, -0.9631, 0.9583
    };

    std::cout << "Original data:" << std::endl;
    std::cout << "0.9835 -0.0581  0.1716 0.0072" << std::endl;
    std::cout << " -0.0937 -0.9738  0.2072 0.0481" << std::endl;
    std::cout << "0.1551 -0.2199 -0.9631 0.9583" << std::endl;
    std::cout << "0 0 0 1" << std::endl;
    std::cout << "M after rotation re-orthogonalization:\n" << M << std::endl;
  }());

  CHECK_NOTHROW([](){
    vpHomogeneousMatrix M1 {
      0.9835, -0.0581,  0.1716, 0.0072,
      -0.0937, -0.9738,  0.2072, 0.0481,
      0.1551, -0.2199, -0.9631, 0.9583
    };

    // if M1 contains a proper rotation matrix
    // following R init should not throw exception since re-orthogonalization
    // is done only in vpHomogeneousMatrix, not in vpRotationMatrix
    vpRotationMatrix R {
      M1[0][0], M1[0][1], M1[0][2],
      M1[1][0], M1[1][1], M1[1][2],
      M1[2][0], M1[2][1], M1[2][2]
    };
  }());

  CHECK_THROWS([](){
    vpHomogeneousMatrix M {
      0.983, -0.058,  0.171, 0.0072,
      -0.093, -0.973,  0.207, 0.0481,
      0.155, -0.219, -0.963, 0.9583
    };
  }());
}

int main(int argc, char *argv[])
{
  Catch::Session session; // There must be exactly one instance

  // Let Catch (using Clara) parse the command line
  session.applyCommandLine(argc, argv);

  int numFailed = session.run();

  // numFailed is clamped to 255 as some unices only use the lower 8 bits.
  // This clamping has already been applied, so just return it here
  // You can also do any post run clean-up here
  return numFailed;
}
#else
#include <iostream>

int main()
{
  return 0;
}
#endif
