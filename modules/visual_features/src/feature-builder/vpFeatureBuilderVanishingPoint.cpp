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
 * Conversion between tracker and visual feature vanishing point.
 *
 * Authors:
 * Odile Bourquardez
 *
 *****************************************************************************/

/*!
  \file vpFeatureBuilderVanishingPoint.cpp
  \brief  conversion between vpPoint
  and visual feature vanishing point.
*/
#include <visp3/core/vpException.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeatureException.h>

/*!
  Initialize a vpFeatureVanishingPoint thanks to a vpPoint.
  The vpFeatureVanishingPoint is initialized thanks to the parameters of the
  point in the image plan. All the parameters are given in meter.

  \param s : Visual feature to initialize.

  \param t : The vpPoint used to create the vpFeatureVanishingPoint.
*/
void vpFeatureBuilder::create(vpFeatureVanishingPoint &s, const vpPoint &t)
{
  try {
    s.set_x(t.get_x());
    s.set_y(t.get_y());
  } catch (...) {
    vpERROR_TRACE("Cannot create vanishing point feature");
    throw;
  }
}

/*!
  Initialize a vpFeatureVanishingPoint thanks to two vpFeatureLine.
  The vpFeatureVanishingPoint is initialized thanks to the coordinate of the
  intersection point in the image plan. All the parameters are given in meter.

  \warning An exception is thrown if the two lines are parallels

  \param s : Visual feature to initialize.

  \param L1 : The first vpFeatureLine.

  \param L2 : The second vpFeatureLine.
*/
void vpFeatureBuilder::create(vpFeatureVanishingPoint &s, const vpFeatureLine &L1, const vpFeatureLine &L2)
{
  double rho_l;
  double rho_r;
  double theta_l;
  double theta_r;
  double c_l;
  double s_l;
  double c_r;
  double s_r;

  rho_l = L1.getRho();
  rho_r = L2.getRho();
  theta_l = L1.getTheta();
  theta_r = L2.getTheta();
  c_l = cos(theta_l);
  c_r = cos(theta_r);
  s_l = sin(theta_l);
  s_r = sin(theta_r);

  double x, y;

  double min = 0.0001;
  if (fabs(theta_r - theta_l) < min || fabs(fabs(theta_r - theta_l) - M_PI) < min ||
      fabs(fabs(theta_r - theta_l) - 2 * M_PI) < min) {
    vpCERROR << "There is no vanishing point : the lines are parallel in the "
                "image plane"
             << std::endl;
    throw(vpFeatureException(vpFeatureException::badInitializationError, "There is no vanishing point : the lines are "
                                                                         "parallel in the image plane"));
  }

  y = (rho_r * c_l - rho_l * c_r) / (-s_l * c_r + s_r * c_l);
  x = (rho_r * s_l - rho_l * s_r) / (-c_l * s_r + c_r * s_l);

  s.set_x(x);
  s.set_y(y);
}

/*!
  Initialize a vpFeatureVanishingPoint thanks to two vpLine.
  The vpFeatureVanishingPoint is initialized thanks to the coordinate of the
  intersection point in the image plan. All the parameters are given in meter.

  \warning An exception is thrown if the two lines are parallels

  \param s : Visual feature to initialize.

  \param L1 : The first vpLine.

  \param L2 : The second vpLine.
*/
void vpFeatureBuilder::create(vpFeatureVanishingPoint &s, const vpLine &L1, const vpLine &L2)
{
  vpFeatureLine l1, l2;
  vpFeatureBuilder::create(l1, L1);
  vpFeatureBuilder::create(l2, L2);

  vpFeatureBuilder::create(s, l1, l2);
}
