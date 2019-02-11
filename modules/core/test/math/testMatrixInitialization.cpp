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
 * Test Matrix initialization.
 *
 *****************************************************************************/

/*!
  \example testMatrixInitialization.cpp

  Test Matrix initialization
*/

#include <visp3/core/vpMatrix.h>

int main()
{
#ifdef VISP_HAVE_CXX11
  {
    vpArray2D<float> a{ 1.f, 2.f, 3.f };
    std::cout << "a:\n" << a << std::endl;
    a = { -1, -2, -3, 4, 5.5, 6.0f };
    std::cout << "a:\n" << a << std::endl;
    a.reshape(2, 3);
    std::cout << "a.reshape(2,3):\n" << a << std::endl;
    a.reshape(3, 2);
    std::cout << "a.reshape(3,2):\n" << a << std::endl;

    vpArray2D<float> a2;
    a2.resize(2,2);
    a2 = {1, 2, 3, 4};
    std::cout << "a2:\n" << a2 << std::endl;

    vpArray2D<double> a3(2, 3, { 1, 2, 3, 4, 5, 6 });
    std::cout << "a3:\n" << a3 << std::endl;

    vpArray2D<int> a4{ {1,2,3},{4,5,6},{7,8,9} };
    std::cout << "a4:\n" << a4 << std::endl;

    vpArray2D<int> a5;
    a5 = { {1,2,3},{4,5,6},{7,8,9} };
    std::cout << "a5:\n" << a5 << std::endl;

    vpArray2D<int> a6{a5};
    std::cout << "a6:\n" << a6 << std::endl;

    vpMatrix m{1, 2, 3};
    std::cout << "m:\n" << m << std::endl;
    m = { -1, -2, -3, -4 };
    std::cout << "m:\n" << m << std::endl;
    m.reshape(2, 2);
    std::cout << "m:\n" << m << std::endl;

    vpMatrix m2(3, 2, { 1, 2, 3, 4, 5, 6 });
    std::cout << "m2:\n" << m2 << std::endl;

    vpMatrix m3{ {1,2,3},{4,5,6},{7,8,9} };
    std::cout << "m3:\n" << m3 << std::endl;

    vpMatrix m4;
    m4 = { {1,2,3},{4,5,6},{7,8,9} };
    std::cout << "m4:\n" << m4 << std::endl;

    vpMatrix m5{m4};
    std::cout << "m5:\n" << m5 << std::endl;

    vpMatrix m6;
    m6 = {m2};
    std::cout << "m6:\n" << m6 << std::endl;

    vpColVector c{ 1, 2, 3 };
    std::cout << "c:\n" << c << std::endl;
    c = { -1, -2, -3 };
    std::cout << "c:\n" << c << std::endl;

    vpRowVector r{ 1, 2, 3, 4, 5, 6 };
    std::cout << "r:\n" << r << std::endl;
    r = { -1, -2, -3 };
    std::cout << "r:\n" << r << std::endl;
  }
#endif

  {
    vpMatrix m1;
    m1 << 1, 2, 3;
    std::cout << "m1:\n" << m1 << std::endl;

    m1 << -1, -2, -3, -4;
    m1.reshape(1, 4);
    std::cout << "m1:\n" << m1 << std::endl;

    vpMatrix m2(2,2);
    m2 << 1, 2, 3, 4, 5, 6, 7, 8, 9;
    std::cout << "m2:\n" << m2 << std::endl;

    m2.resize(3, 3, false);
    std::cout << "m2:\n" << m2 << std::endl;

    m2 << 0.0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11;
    m2.reshape(2, 6);
    std::cout << "m2:\n" << m2 << std::endl;

    vpColVector c;
    c << 1, 2, 3, 4;
    std::cout << "c: " << c << std::endl;

    try {
      c.reshape(2, 2);
      std::cout << "after c.reshape(2, 2): " << c.t() << std::endl;
      c = c.reshape(2, 2);
      std::cout << "c:" << c << std::endl;
    } catch (const vpException &e) {
      std::cerr << "Exception: c = c.reshape(2, 2);\n" << e.what() << std::endl;
    }

    vpRowVector r;
    r << 1, 2, 3;
    std::cout << "r: " << r << std::endl;

    vpMatrix m = r.reshape(3, 1);
    std::cout << "m:\n" << m << std::endl;

    try {
      r.reshape(3, 1);
      std::cout << "after r.reshape(3, 1): " << r << std::endl;
    } catch (const vpException &e) {
      std::cerr << "Exception: r.reshape(3, 1);\n" << e.what() << std::endl;
    }

    std::cout << "c: " << c.t() << std::endl;
    vpArray2D<double> * ptr_array = &c;
    ptr_array->reshape(2,2);
    std::cout << "ptr_array->reshape(2,2)" << std::endl;
    std::cout << "c: (" << c.getRows() << ", " << c.getCols() << "):\n" << c << std::endl;
    std::cout << "dynamic_cast<vpColVector *>(ptr_array):\n" << *dynamic_cast<vpColVector *>(ptr_array) << std::endl;
    std::cout << "ptr_array:\n" << *ptr_array << std::endl;
  }

  return  EXIT_SUCCESS;
}
