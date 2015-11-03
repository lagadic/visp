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
  \example testColVector.cpp

  Test some vpColVector functionalities.
*/

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpDebug.h>

#include <stdlib.h>
#include <stdio.h>


int main()
{
  try {
    vpColVector V(4) ;
    V = 1.0;

    vpTRACE("------------------------");
    vpTRACE("call std::cout << V;");
    std::cout << V << std::endl;

    vpTRACE("------------------------");
    vpTRACE("call V.normalize();");
    V.normalize();

    vpTRACE("------------------------");
    vpTRACE("call std::cout << V;");
    std::cout << V << std::endl;


    //Test stack
    vpColVector col_vector1(4);
    col_vector1 = 1.0;
    col_vector1.stack(2.0);
    std::cout << "col_vector1.stack(2.0)=\n" << col_vector1 << std::endl;

    vpColVector col_vector2(3);
    col_vector2 = 5.0;

    vpColVector col_vector3;
    col_vector3 = vpColVector::stack(col_vector1, col_vector2);
    std::cout << "vpColVector::stack(col_vector1, col_vector2)=\n" << col_vector3 << std::endl;

    col_vector1.stack(col_vector2);
    std::cout << "col_vector1.stack(col_vector2)=\n" << col_vector1 << std::endl;


    //Test mean, median and standard deviation against Matlab with rng(0) and rand(10,1)*10
    vpColVector colVector(10);
    colVector[0] = 8.1472;
    colVector[1] = 9.0579;
    colVector[2] = 1.2699;
    colVector[3] = 9.1338;
    colVector[4] = 6.3236;
    colVector[5] = 0.9754;
    colVector[6] = 2.7850;
    colVector[7] = 5.4688;
    colVector[8] = 9.5751;
    colVector[9] = 9.6489;


    double res = vpColVector::mean(colVector);
    if(!vpMath::equal(res, 6.2386, 0.001)) {
      std::cerr << "Problem with vpColVector::mean()=" << res << std::endl;
      return -1;
    }
    std::cout << "vpColVector::mean() is Ok !" << std::endl;

    res = vpColVector::stdev(colVector);
    if(!vpMath::equal(res, 3.2810, 0.001)) {
      std::cerr << "Problem with vpColVector::stdev()=" << res << std::endl;
      return -1;
    }

    res = vpColVector::stdev(colVector, true);
    if(!vpMath::equal(res, 3.4585, 0.001)) {
      std::cerr << "Problem with vpColVector::stdev() with Bessel correction=" << res << std::endl;
      return -1;
    }
    std::cout << "vpColVector::stdev() is Ok !" << std::endl;

    res = vpColVector::median(colVector);
    if(!vpMath::equal(res, 7.2354, 0.001)) {
      std::cerr << "Problem with vpColVector::median()=" << res << std::endl;
      return -1;
    }

    //Test median with odd number of elements
    colVector.stack(1.5761);
    res = vpColVector::median(colVector);
    if(!vpMath::equal(res, 6.3236, 0.001)) {
      std::cerr << "Problem with vpColVector::median()=" << res << std::endl;
      return -1;
    }
    std::cout << "vpColVector::median() is Ok !" << std::endl;

    std::cout << "OK !" << std::endl;
    return (0);
  }
  catch(vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return (1);
  }
}
