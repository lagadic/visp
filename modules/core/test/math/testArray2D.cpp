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
  \example testArray2D.cpp

  Test some vpArray2D functionalities.
*/

#include <cmath>
#include <vector>
#include <limits>

#include <visp3/core/vpTranslationVector.h>

template<typename Type>
bool test(const std::string &s, const vpArray2D<Type> &A, const std::vector<Type> &bench)
{
  static unsigned int cpt = 0;
  std::cout << "** Test " << ++cpt << std::endl;
  std::cout << s << "(" << A.getRows() << "," << A.getCols() << ") = \n" << A << std::endl;
  if(bench.size() != A.size()) {
    std::cout << "Test fails: bad size wrt bench" << std::endl;
    return false;
  }
  for (unsigned int i=0; i<A.size(); i++) {
    if (std::fabs(A.data[i]-bench[i]) > std::fabs(A.data[i])*std::numeric_limits<double>::epsilon()) {
      std::cout << "Test fails: bad content" << std::endl;
      return false;
    }
  }

  return true;
}

int main()
{
  int err = 1;
  {
    // test default constructor
    vpArray2D<double> A;
    std::vector<double> bench;
    if (test("A", A, bench) == false)
      return err;
  }
  {
    // test copy constructor
    vpArray2D<double> A(3, 4);

    std::vector<double> bench(12);
    for(unsigned int i=0; i<3; i++) {
      for(unsigned int j=0; j<4; j++) {
        A[i][j] = (double)(i+j);
        bench[i*4+j] = (double)(i+j);
      }
    }
    if (test("A", A, bench) == false)
      return err;

    vpArray2D<double> B(A);
    if (test("B", B, bench) == false)
      return err;
    std::cout << "Min/Max: " << B.getMinValue() << " " << B.getMaxValue() << std::endl;
  }
  {
    // test constructor with initial value
    vpArray2D<double> A(3, 4, 2.);
    std::vector<double> bench1(12, 2);
    if (test("A", A, bench1) == false)
      return err;

    A.resize(5, 6);
    std::vector<double> bench2(30, 0);
    if (test("A", A, bench2) == false)
      return err;

    A = -2.;
    std::vector<double> bench3(30, -2);
    if (test("A", A, bench3) == false)
      return err;
  }

  // Test with float
  {
    // test default constructor
    vpArray2D<float> A;
    std::vector<float> bench;
    if (test("A", A, bench) == false)
      return err;
  }
  {
    // test copy constructor
    vpArray2D<float> A(3, 4);

    std::vector<float> bench(12);
    for(unsigned int i=0; i<3; i++) {
      for(unsigned int j=0; j<4; j++) {
        A[i][j] = (float)(i+j);
        bench[i*4+j] = (float)(i+j);
      }
    }
    if (test("A", A, bench) == false)
      return err;

    vpArray2D<float> B(A);
    if (test("B", B, bench) == false)
      return err;
    std::cout << "Min/Max: " << B.getMinValue() << " " << B.getMaxValue() << std::endl;
  }
  {
    // test constructor with initial value
    vpArray2D<float> A(3, 4, 2.);
    std::vector<float> bench1(12, 2);
    if (test("A", A, bench1) == false)
      return err;

    A.resize(5, 6);
    std::vector<float> bench2(30, 0);
    if (test("A", A, bench2) == false)
      return err;

    A = -2.;
    std::vector<float> bench3(30, -2);
    if (test("A", A, bench3) == false)
      return err;
  }
  std::cout << "All tests succeed" << std::endl;
  return 0;
}
