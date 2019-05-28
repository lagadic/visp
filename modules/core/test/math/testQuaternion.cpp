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
 * Tests quaternion operations.
 *
 * Author:
 * Souriya Trinh
 *
 *****************************************************************************/

/*!
  \file testQuaternion.cpp
  \brief Tests quaternion operations.
*/

#include <limits>
#include <visp3/core/vpException.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpQuaternionVector.h>

int main()
{
  try {
    // Test addition of two quaternions
    vpQuaternionVector q1(2.1, -1, -3.7, 1.5);
    vpQuaternionVector q2(0.5, 1.4, 0.7, 2.5);
    vpQuaternionVector q3 = q1 + q2;
    std::cout << "q3=" << q3 << std::endl;
    if (!vpMath::equal(q3.x(), 2.6, std::numeric_limits<double>::epsilon()) ||
        !vpMath::equal(q3.y(), 0.4, std::numeric_limits<double>::epsilon()) ||
        !vpMath::equal(q3.z(), -3.0, std::numeric_limits<double>::epsilon()) ||
        !vpMath::equal(q3.w(), 4.0, std::numeric_limits<double>::epsilon())) {
      throw vpException(vpException::fatalError, "Problem with addition of two quaternions !");
    }

    // Test subtraction of two quaternions
    vpQuaternionVector q4 = q3 - q1;
    std::cout << "q4=" << q4 << std::endl;
    if (!vpMath::equal(q4.x(), q2.x(), std::numeric_limits<double>::epsilon() * 1e4) ||
        !vpMath::equal(q4.y(), q2.y(), std::numeric_limits<double>::epsilon() * 1e4) ||
        !vpMath::equal(q4.z(), q2.z(), std::numeric_limits<double>::epsilon() * 1e4) ||
        !vpMath::equal(q4.w(), q2.w(), std::numeric_limits<double>::epsilon() * 1e4)) {
      throw vpException(vpException::fatalError, "Problem with subtraction of two quaternions !");
    }

    // Test multiplication of two quaternions
    // https://www.wolframalpha.com/input/?i=quaternion+-Sin%5BPi%5D%2B3i%2B4j%2B3k+multiplied+by+-1j%2B3.9i%2B4-3k&lk=3
    vpQuaternionVector q5(3.0, 4.0, 3.0, -sin(M_PI));
    vpQuaternionVector q6(3.9, -1.0, -3.0, 4.0);
    vpQuaternionVector q7 = q5 * q6;
    std::cout << "q7=" << q7 << std::endl;
    if (!vpMath::equal(q7.x(), 3.0, std::numeric_limits<double>::epsilon() * 1e4) ||
        !vpMath::equal(q7.y(), 36.7, std::numeric_limits<double>::epsilon() * 1e4) ||
        !vpMath::equal(q7.z(), -6.6, std::numeric_limits<double>::epsilon() * 1e4) ||
        !vpMath::equal(q7.w(), 1.3, std::numeric_limits<double>::epsilon() * 1e4)) {
      throw vpException(vpException::fatalError, "Problem with multiplication of two quaternions !");
    }

    // Test quaternion conjugate
    vpQuaternionVector q7_conj = q7.conjugate();
    std::cout << "q7_conj=" << q7_conj << std::endl;
    if (!vpMath::equal(q7_conj.x(), -3.0, std::numeric_limits<double>::epsilon() * 1e4) ||
        !vpMath::equal(q7_conj.y(), -36.7, std::numeric_limits<double>::epsilon() * 1e4) ||
        !vpMath::equal(q7_conj.z(), 6.6, std::numeric_limits<double>::epsilon() * 1e4) ||
        !vpMath::equal(q7_conj.w(), 1.3, std::numeric_limits<double>::epsilon() * 1e4)) {
      throw vpException(vpException::fatalError, "Problem with quaternion conjugate !");
    }

    // Test quaternion inverse
    vpQuaternionVector q7_inv = q7.inverse();
    std::cout << "q7_inv=" << q7_inv << std::endl;
    if (!vpMath::equal(q7_inv.x(), -0.00214111, 0.000001) || !vpMath::equal(q7_inv.y(), -0.026193, 0.000001) ||
        !vpMath::equal(q7_inv.z(), 0.00471045, 0.000001) || !vpMath::equal(q7_inv.w(), 0.000927816, 0.000001)) {
      throw vpException(vpException::fatalError, "Problem with quaternion inverse !");
    }

    // Test quaternion norm
    double q7_norm = q7.magnitude();
    std::cout << "q7_norm=" << q7_norm << std::endl;
    if (!vpMath::equal(q7_norm, 37.4318, 0.0001)) {
      throw vpException(vpException::fatalError, "Problem with quaternion magnitude !");
    }

    // Test quaternion normalization
    q7.normalize();
    std::cout << "q7_unit=" << q7 << std::endl;
    if (!vpMath::equal(q7.x(), 0.0801457, 0.00001) || !vpMath::equal(q7.y(), 0.98045, 0.00001) ||
        !vpMath::equal(q7.z(), -0.176321, 0.00001) || !vpMath::equal(q7.w(), 0.0347298, 0.00001)) {
      throw vpException(vpException::fatalError, "Problem with quaternion normalization !");
    }

    // Test copy constructor
    vpQuaternionVector q_copy1 = vpQuaternionVector(0, 0, 1, 1);
    std::cout << "q_copy1=" << q_copy1 << std::endl;
    vpQuaternionVector q_copy2 = q_copy1;
    q_copy1.set(1, 0, 1, 10);
    std::cout << "q_copy1 after set=" << q_copy1 << std::endl;
    std::cout << "q_copy2=" << q_copy2 << std::endl;

    // Test assignment operator
    vpQuaternionVector q_copy3(10, 10, 10, 10);
    q_copy3 = q_copy1;
    std::cout << "q_copy3=" << q_copy3 << std::endl;

    std::cout << "vpQuaternion operations are ok !" << std::endl;
    return 0;
  } catch (const vpException &e) {
    std::cerr << "Catch an exception: " << e << std::endl;
    return 1;
  }
}
