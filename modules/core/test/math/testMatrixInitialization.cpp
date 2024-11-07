/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
 * See https://visp.inria.fr for more information.
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
 * Test Matrix initialization.
 */

/*!
  \example testMatrixInitialization.cpp

  Test Matrix initialization
*/

#include <visp3/core/vpMatrix.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

bool equal(const vpArray2D<double> &a1, const vpArray2D<double> &a2, double epsilon)
{
  if (a1.size() != a2.size()) {
    std::cout << "Rotation vector size differ" << std::endl;
    return false;
  }
  for (unsigned int i = 0; i < a1.getRows(); i++) {
    for (unsigned int j = 0; j < a1.getCols(); j++) {
      if (!vpMath::equal(a1[i][j], a2[i][j], epsilon)) {
        std::cout << "Array content differ" << std::endl;
        return false;
      }
    }
  }
  return true;
}

bool equal(const vpRotationVector &a1, const vpRotationVector &a2, double epsilon)
{
  if (a1.size() != a2.size()) {
    std::cout << "Rotation vector size differ" << std::endl;
    return false;
  }
  for (unsigned int i = 0; i < a1.size(); i++) {
    if (!vpMath::equal(a1[i], a2[i], epsilon)) {
      std::cout << "Rotation vector content differ" << std::endl;
      return false;
    }
  }
  return true;
}

bool equal(const vpColVector &a1, const vpColVector &a2, double epsilon)
{
  if (a1.size() != a2.size()) {
    std::cout << "Column vector size differ" << std::endl;
    return false;
  }
  for (unsigned int i = 0; i < a1.size(); i++) {
    if (!vpMath::equal(a1[i], a2[i], epsilon)) {
      std::cout << "Column vector content differ" << std::endl;
      return false;
    }
  }
  return true;
}

bool equal(const vpRowVector &a1, const vpRowVector &a2, double epsilon)
{
  if (a1.size() != a2.size()) {
    std::cout << "Row vector size differ" << std::endl;
    return false;
  }
  for (unsigned int i = 0; i < a1.size(); i++) {
    if (!vpMath::equal(a1[i], a2[i], epsilon)) {
      std::cout << "Row vector content differ" << std::endl;
      return false;
    }
  }
  return true;
}

int main()
{
  double epsilon = 1e-10;

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  {
    vpArray2D<float> a { 1.f, 2.f, 3.f };
    std::cout << "a:\n" << a << std::endl;
    a = { -1, -2, -3, 4, 5.5, 6.0f };
    std::cout << "a:\n" << a << std::endl;
    a.reshape(2, 3);
    std::cout << "a.reshape(2,3):\n" << a << std::endl;
    a.reshape(3, 2);
    std::cout << "a.reshape(3,2):\n" << a << std::endl;

    vpArray2D<float> a2;
    a2.resize(2, 2);
    a2 = { 1, 2, 3, 4 };
    std::cout << "a2:\n" << a2 << std::endl;

    vpArray2D<double> a3(2, 3, { 1, 2, 3, 4, 5, 6 });
    std::cout << "a3:\n" << a3 << std::endl;

    vpArray2D<int> a4 { {1, 2, 3}, {4, 5, 6}, {7, 8, 9} };
    std::cout << "a4:\n" << a4 << std::endl;

    vpArray2D<int> a5;
    a5 = { {1, 2, 3}, {4, 5, 6}, {7, 8, 9} };
    std::cout << "a5:\n" << a5 << std::endl;

    vpArray2D<int> a6 { a5 };
    std::cout << "a6:\n" << a6 << std::endl;

    vpMatrix m { 1, 2, 3 };
    std::cout << "m:\n" << m << std::endl;
    m = { -1, -2, -3, -4 };
    std::cout << "m:\n" << m << std::endl;
    m.reshape(2, 2);
    std::cout << "m:\n" << m << std::endl;

    vpMatrix m2(3, 2, { 1, 2, 3, 4, 5, 6 });
    std::cout << "m2:\n" << m2 << std::endl;

    vpMatrix m3 { {1, 2, 3}, {4, 5, 6}, {7, 8, 9} };
    std::cout << "m3:\n" << m3 << std::endl;

    vpMatrix m4;
    m4 = { {1, 2, 3}, {4, 5, 6}, {7, 8, 9} };
    std::cout << "m4:\n" << m4 << std::endl;

    vpMatrix m5 { m4 };
    std::cout << "m5:\n" << m5 << std::endl;

    //    vpMatrix m6;
    //    m6 = {m2};  // Fails on travis
    //    std::cout << "m6:\n" << m6 << std::endl;
  }
#endif

  {
    vpMatrix m1;
    m1 << 1, 2, 3;
    std::cout << "m1:\n" << m1 << std::endl;

    m1 << -1, -2, -3, -4;
    m1.reshape(1, 4);
    std::cout << "m1:\n" << m1 << std::endl;

    vpMatrix m2(2, 2);
    m2 << 1, 2, 3, 4, 5, 6, 7, 8, 9;
    std::cout << "m2:\n" << m2 << std::endl;

    m2.resize(3, 3, false);
    std::cout << "m2:\n" << m2 << std::endl;

    m2 << 0.0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11;
    m2.reshape(2, 6);
    std::cout << "m2:\n" << m2 << std::endl;
  }

  {
    std::cout << "** Test vpColVector" << std::endl;
    vpColVector c_ref(6);
    for (unsigned int i = 0; i < 6; i++) {
      c_ref[i] = i;
    }
    std::cout << "c_ref: " << c_ref.t() << std::endl;
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
    {
      vpColVector c { 0, 1, 2, 3, 4, 5 };
      std::cout << "c: " << c.t() << std::endl;
      if (!equal(c_ref, c, epsilon)) {
        return EXIT_FAILURE;
      }
      c_ref.resize(3, false);
      c_ref *= -1;
      std::cout << "c_ref: " << c_ref.t() << std::endl;
      c = { 0, -1, -2 };
      std::cout << "c: " << c.t() << std::endl;
      if (!equal(c_ref, c, epsilon)) {
        return EXIT_FAILURE;
      }

      // Test move constructor
      vpColVector c1(c_ref);
      std::cout << "c1: " << c1.t() << std::endl;
      if (!equal(c_ref, c1, epsilon)) {
        return EXIT_FAILURE;
      }
      vpColVector c2 = std::move(c1); // Move c1 into c2; c1 is now "empty"
      std::cout << "c1: " << c1.t() << std::endl;
      if (c1.size()) {
        return EXIT_FAILURE;
      }
      std::cout << "c2: " << c2.t() << std::endl;
      if (!equal(c_ref, c2, epsilon)) {
        return EXIT_FAILURE;
      }
    }
#endif
    {
      vpColVector c;
      c << 1, 2, 3, 4;
      std::cout << "c: " << c << std::endl;

      try {
        c.reshape(2, 2);
        std::cout << "after c.reshape(2, 2): " << c.t() << std::endl;
        c = c.reshape(2, 2);
        std::cout << "c:" << c << std::endl;
      }
      catch (const vpException &e) {
        std::cerr << "Exception expected: c = c.reshape(2, 2);\n" << e.what() << std::endl;
      }

      std::cout << "c: " << c.t() << std::endl;
      vpArray2D<double> *ptr_array = &c;
      ptr_array->reshape(2, 2);
      std::cout << "ptr_array->reshape(2,2)" << std::endl;
      std::cout << "c: (" << c.getRows() << ", " << c.getCols() << "):\n" << c << std::endl;
      std::cout << "dynamic_cast<vpColVector *>(ptr_array):\n" << *dynamic_cast<vpColVector *>(ptr_array) << std::endl;
      std::cout << "ptr_array:\n" << *ptr_array << std::endl;
    }
  }

  {
    std::cout << "** Test vpRowVector" << std::endl;
    vpRowVector r_ref(6);
    for (unsigned int i = 0; i < 6; i++) {
      r_ref[i] = i;
    }
    std::cout << "r_ref: " << r_ref << std::endl;
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
    {
      vpRowVector r { 0, 1, 2, 3, 4, 5 };
      std::cout << "r: " << r << std::endl;
      if (!equal(r_ref, r, epsilon)) {
        return EXIT_FAILURE;
      }
      r_ref.resize(3, false);
      r_ref *= -1;
      std::cout << "r_ref: " << r_ref << std::endl;
      r = { 0, -1, -2 };
      std::cout << "r: " << r << std::endl;
      if (!equal(r_ref, r, epsilon)) {
        return EXIT_FAILURE;
      }

      // Test move constructor
      vpRowVector r1(r_ref);
      std::cout << "r1: " << r1 << std::endl;
      if (!equal(r_ref, r1, epsilon)) {
        return EXIT_FAILURE;
      }
      vpRowVector r2 = std::move(r1); // Move r1 into r2; r1 is now "empty"
      std::cout << "r1: " << r1 << std::endl;
      if (r1.size()) {
        return EXIT_FAILURE;
      }
      std::cout << "r2: " << r2 << std::endl;
      if (!equal(r_ref, r2, epsilon)) {
        return EXIT_FAILURE;
      }
    }
#endif
    {
      vpRowVector r;
      r << 1, 2, 3;
      std::cout << "r: " << r << std::endl;

      vpMatrix m = r.reshape(3, 1);
      std::cout << "m:\n" << m << std::endl;

      try {
        r.reshape(3, 1);
        std::cout << "after r.reshape(3, 1): " << r << std::endl;
      }
      catch (const vpException &e) {
        std::cerr << "Exception: r.reshape(3, 1);\n" << e.what() << std::endl;
      }
    }
  }

  {
    std::cout << "** Test vpThetaUVector" << std::endl;
    vpThetaUVector tu_ref(0, M_PI_2, M_PI);
    std::cout << "tu_ref: " << tu_ref.t() << std::endl;
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
    {
      vpThetaUVector tu = { 0, M_PI_2, M_PI };
      std::cout << "tu: " << tu.t() << std::endl;
      if (!equal(tu_ref, tu, epsilon)) {
        return EXIT_FAILURE;
      }
    }
#endif
    {
      vpThetaUVector tu;
      tu << 0, M_PI_2, M_PI;
      std::cout << "tu: " << tu.t() << std::endl;
      if (!equal(tu_ref, tu, epsilon)) {
        return EXIT_FAILURE;
      }
      // Do it twice
      tu << 0, M_PI_2, M_PI;
      std::cout << "tu: " << tu.t() << std::endl;
      if (!equal(tu_ref, tu, epsilon)) {
        return EXIT_FAILURE;
      }
    }
  }

  {
    std::cout << "** Test vpRxyzVector" << std::endl;
    vpRxyzVector rxyz_ref(0, M_PI_2, M_PI);
    std::cout << "rxyz_ref: " << rxyz_ref.t() << std::endl;
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
    {
      vpRxyzVector rxyz = { 0, M_PI_2, M_PI };
      std::cout << "rxyz: " << rxyz.t() << std::endl;
      if (!equal(rxyz_ref, rxyz, epsilon)) {
        return EXIT_FAILURE;
      }
    }
#endif
    {
      vpRxyzVector rxyz;
      rxyz << 0, M_PI_2, M_PI;
      std::cout << "rxyz: " << rxyz.t() << std::endl;
      if (!equal(rxyz_ref, rxyz, epsilon)) {
        return EXIT_FAILURE;
      }
      // Do it twice
      rxyz << 0, M_PI_2, M_PI;
      std::cout << "rxyz: " << rxyz.t() << std::endl;
      if (!equal(rxyz_ref, rxyz, epsilon)) {
        return EXIT_FAILURE;
      }
    }
  }

  {
    std::cout << "** Test vpRzyxVector" << std::endl;
    vpRzyxVector rzyx_ref(0, M_PI_2, M_PI);
    std::cout << "rzyx_ref: " << rzyx_ref.t() << std::endl;
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
    {
      vpRzyxVector rzyx = { 0, M_PI_2, M_PI };
      std::cout << "rzyx: " << rzyx.t() << std::endl;
      if (!equal(rzyx_ref, rzyx, epsilon)) {
        return EXIT_FAILURE;
      }
    }
#endif
    {
      vpRzyxVector rzyx;
      rzyx << 0, M_PI_2, M_PI;
      std::cout << "rzyx: " << rzyx.t() << std::endl;
      if (!equal(rzyx_ref, rzyx, epsilon)) {
        return EXIT_FAILURE;
      }
      // Do it twice
      rzyx << 0, M_PI_2, M_PI;
      std::cout << "rzyx: " << rzyx.t() << std::endl;
      if (!equal(rzyx_ref, rzyx, epsilon)) {
        return EXIT_FAILURE;
      }
    }
  }

  {
    std::cout << "** Test vpRzyzVector" << std::endl;
    vpRzyzVector rzyz_ref(0, M_PI_2, M_PI);
    std::cout << "rzyz_ref: " << rzyz_ref.t() << std::endl;
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
    {
      vpRzyzVector rzyz = { 0, M_PI_2, M_PI };
      std::cout << "rzyz: " << rzyz.t() << std::endl;
      if (!equal(rzyz_ref, rzyz, epsilon)) {
        return EXIT_FAILURE;
      }
    }
#endif
    {
      vpRzyzVector rzyz;
      rzyz << 0, M_PI_2, M_PI;
      std::cout << "rzyz: " << rzyz.t() << std::endl;
      if (!equal(rzyz_ref, rzyz, epsilon)) {
        return EXIT_FAILURE;
      }
      // Do it twice
      rzyz << 0, M_PI_2, M_PI;
      std::cout << "rzyz: " << rzyz.t() << std::endl;
      if (!equal(rzyz_ref, rzyz, epsilon)) {
        return EXIT_FAILURE;
      }
    }
  }

  {
    std::cout << "** Test vpQuaternionVector" << std::endl;
    vpThetaUVector tu_ref(0, M_PI_2, M_PI);
    vpQuaternionVector q_ref(tu_ref);
    std::cout << "q_ref: " << q_ref.t() << std::endl;
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
    {
      vpQuaternionVector q = { q_ref[0], q_ref[1], q_ref[2], q_ref[3] };
      std::cout << "q: " << q.t() << std::endl;
      if (!equal(q_ref, q, epsilon)) {
        return EXIT_FAILURE;
      }
    }
#endif
    {
      vpQuaternionVector q;
      q << q_ref[0], q_ref[1], q_ref[2], q_ref[3];
      std::cout << "q: " << q.t() << std::endl;
      if (!equal(q_ref, q, epsilon)) {
        return EXIT_FAILURE;
      }
      // Do it twice
      q << q_ref[0], q_ref[1], q_ref[2], q_ref[3];
      std::cout << "q: " << q.t() << std::endl;
      if (!equal(q_ref, q, epsilon)) {
        return EXIT_FAILURE;
      }
    }
  }

  {
    std::cout << "** Test vpTranslationVector" << std::endl;
    vpTranslationVector t_ref(0, 0.1, 0.5);
    std::cout << "t_ref: " << t_ref.t() << std::endl;
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
    {
      vpTranslationVector t = { t_ref[0], t_ref[1], t_ref[2] };
      std::cout << "t: " << t.t() << std::endl;
      if (!equal(t_ref, t, epsilon)) {
        return EXIT_FAILURE;
      }
    }
#endif
    {
      vpTranslationVector t;
      t << 0, 0.1, 0.5;
      std::cout << "t: " << t.t() << std::endl;
      if (!equal(t_ref, t, epsilon)) {
        return EXIT_FAILURE;
      }
      // Do it twice
      t << 0, 0.1, 0.5;
      std::cout << "t: " << t.t() << std::endl;
      if (!equal(t_ref, t, epsilon)) {
        return EXIT_FAILURE;
      }
    }
  }

  {
    std::cout << "** Test vpRotationMatrix" << std::endl;
    vpRotationMatrix R_ref(vpRxyzVector(0, -M_PI_2, M_PI));
    std::cout << "R_ref:\n" << R_ref << std::endl;
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
    {
      vpRotationMatrix R({ 0, 0, -1, 0, -1, 0, -1, 0, 0 });
      std::cout << "R:\n" << R << std::endl;
      if (!equal(R_ref, R, epsilon)) {
        return EXIT_FAILURE;
      }
    }
    {
      vpRotationMatrix R;
      R = { 0, 0, -1, 0, -1, 0, -1, 0, 0 };
      std::cout << "R:\n" << R << std::endl;
      if (!equal(R_ref, R, epsilon)) {
        return EXIT_FAILURE;
      }
    }
#endif
    {
      vpRotationMatrix R;
      R << 0, 0, -1, 0, -1, 0, -1, 0, 0;
      std::cout << "R:\n" << R << std::endl;
      if (!equal(R_ref, R, epsilon)) {
        return EXIT_FAILURE;
      }
      // Do it twice
      R << 0, 0, -1, 0, -1, 0, -1, 0, 0;
      std::cout << "R:\n" << R << std::endl;
      if (!equal(R_ref, R, epsilon)) {
        return EXIT_FAILURE;
      }
    }
  }

  std::cout << "Test succeed" << std::endl;
  return EXIT_SUCCESS;
}
