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
  \example testRowVector.cpp

  Test some vpRowVector functionalities.
*/

#include <visp3/core/vpRowVector.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpMath.h>

#include <stdlib.h>
#include <stdio.h>


int main()
{
  try {
    vpRowVector V(4) ;
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
    vpRowVector row_vector1(4);
    row_vector1 = 1.0;
    row_vector1.stack(2.0);
    std::cout << "row_vector1.stack(2.0)=" << row_vector1 << std::endl;

    vpRowVector row_vector2(3);
    row_vector2 = 5.0;

    vpRowVector row_vector3;
    row_vector3 = vpRowVector::stack(row_vector1, row_vector2);
    std::cout << "vpRowVector::stack(row_vector1, row_vector2)=" << row_vector3 << std::endl;

    row_vector1.stack(row_vector2);
    std::cout << "row_vector1.stack(row_vector2)=" << row_vector1 << std::endl;


    //Test mean, median and standard deviation against Matlab with rng(0) and rand(10,1)*10
    vpRowVector rowVector(10);
    rowVector[0] = 8.1472;
    rowVector[1] = 9.0579;
    rowVector[2] = 1.2699;
    rowVector[3] = 9.1338;
    rowVector[4] = 6.3236;
    rowVector[5] = 0.9754;
    rowVector[6] = 2.7850;
    rowVector[7] = 5.4688;
    rowVector[8] = 9.5751;
    rowVector[9] = 9.6489;


    double res = vpRowVector::mean(rowVector);
    if(!vpMath::equal(res, 6.2386, 0.001)) {
      std::cerr << "Problem with vpRowVector::mean()=" << res << std::endl;
      return -1;
    }
    std::cout << "vpRowVector::mean() is Ok !" << std::endl;

    res = vpRowVector::stdev(rowVector);
    if(!vpMath::equal(res, 3.2810, 0.001)) {
      std::cerr << "Problem with vpRowVector::stdev()=" << res << std::endl;
      return -1;
    }

    res = vpRowVector::stdev(rowVector, true);
    if(!vpMath::equal(res, 3.4585, 0.001)) {
      std::cerr << "Problem with vpRowVector::stdev() with Bessel correction=" << res << std::endl;
      return -1;
    }
    std::cout << "vpRowVector::stdev() is Ok !" << std::endl;

    res = vpRowVector::median(rowVector);
    if(!vpMath::equal(res, 7.2354, 0.001)) {
      std::cerr << "Problem with vpColVector::median()=" << res << std::endl;
      return -1;
    }

    //Test median with odd number of elements
    rowVector.stack(1.5761);
    res = vpRowVector::median(rowVector);
    if(!vpMath::equal(res, 6.3236, 0.001)) {
      std::cerr << "Problem with vpRowVector::median()=" << res << std::endl;
      return -1;
    }
    std::cout << "vpRowVector::median() is Ok !" << std::endl;

    std::cout << "OK !" << std::endl;
    return (0);
  }
  catch(vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return (1);
  }
}
