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
 * Performs various tests on the point class.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

/*!
  \file testPoint.cpp
  \brief Performs various tests on the the point class.
*/

#include <visp3/core/vpDebug.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpPoint.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeatureException.h>
#include <visp3/visual_features/vpFeaturePoint.h>

#include <stdio.h>
#include <stdlib.h>

int main()
{
  try {
    vpHomogeneousMatrix cMo;
    cMo[0][3] = 0.1;
    cMo[1][3] = 0.2;
    cMo[2][3] = 2;

    vpPoint point;
    vpTRACE("set point coordinates in the world  frame ");
    point.setWorldCoordinates(0, 0, 0);

    std::cout << "------------------------------------------------------" << std::endl;
    vpTRACE("test the projection ");
    point.track(cMo);

    vpTRACE("coordinates in the world frame ");
    std::cout << point.oP.t() << std::endl;
    vpTRACE("coordinates in the camera frame  ");
    std::cout << point.cP.t() << std::endl;

    vpTRACE("2D coordinates ");
    std::cout << point.get_x() << "  " << point.get_y() << std::endl;

    std::cout << "------------------------------------------------------" << std::endl;
    vpTRACE("test the interaction matrix ");

    vpFeaturePoint p;
    vpFeatureBuilder::create(p, point);

    vpMatrix L;
    L = p.interaction();
    std::cout << L << std::endl;

    vpTRACE("test the interaction matrix select");
    vpTRACE("\t only X");
    L = p.interaction(vpFeaturePoint::selectX());
    std::cout << L << std::endl;

    vpTRACE("\t only Y");
    L = p.interaction(vpFeaturePoint::selectY());
    std::cout << L << std::endl;

    vpTRACE("\t X & Y");
    L = p.interaction(vpFeaturePoint::selectX() | vpFeaturePoint::selectY());
    std::cout << L << std::endl;

    vpTRACE("\t selectAll");
    L = p.interaction(vpFeaturePoint::selectAll());
    std::cout << L << std::endl;

    std::cout << "------------------------------------------------------" << std::endl;
    vpTRACE("test the error ");

    try {
      vpFeaturePoint pd;
      pd.set_x(0);
      pd.set_y(0);

      pd.print();
      std::cout << std::endl;
      vpColVector e;
      e = p.error(pd);
      std::cout << e << std::endl;

      vpTRACE("test the interaction matrix select");
      vpTRACE("\t only X");
      e = p.error(pd, vpFeaturePoint::selectX());
      std::cout << e << std::endl;

      vpTRACE("\t only Y");
      e = p.error(pd, vpFeaturePoint::selectY());
      std::cout << e << std::endl;

      vpTRACE("\t X & Y");
      e = p.error(pd, vpFeaturePoint::selectX() | vpFeaturePoint::selectY());
      std::cout << e << std::endl;

      vpTRACE("\t selectAll");
      e = p.error(pd, vpFeaturePoint::selectAll());
      std::cout << e << std::endl;
    } catch (vpFeatureException &me) {
      std::cout << me << std::endl;
    } catch (const vpException &me) {
      std::cout << me << std::endl;
    }
    std::cout << "------------------------------------------------------" << std::endl;
    vpTRACE("test the  dimension");
    unsigned int dim;
    dim = p.getDimension();
    std::cout << "Dimension = " << dim << std::endl;

    vpTRACE("test the dimension with  select");
    vpTRACE("\t only X");
    dim = p.getDimension(vpFeaturePoint::selectX());
    std::cout << "Dimension = " << dim << std::endl;

    vpTRACE("\t only Y");
    dim = p.getDimension(vpFeaturePoint::selectY());
    std::cout << "Dimension = " << dim << std::endl;

    vpTRACE("\t X & Y");
    dim = p.getDimension(vpFeaturePoint::selectX() | vpFeaturePoint::selectY());
    std::cout << "Dimension = " << dim << std::endl;

    vpTRACE("\t selectAll");
    dim = p.getDimension(vpFeaturePoint::selectAll());
    std::cout << "Dimension = " << dim << std::endl;
    return 0;
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return 1;
  }
}
