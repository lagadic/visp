/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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

BEGIN_VISP_NAMESPACE
/*!
  Initialize a vpFeatureVanishingPoint thanks to a vpPoint.
  The vpFeatureVanishingPoint is initialized thanks to the parameters of the
  point in the image plane. All the parameters are given in meter.

  \param s : Visual feature to initialize; either \f$ {\bf s} = (x, y) \f$ or either
  \f$ {\bf s} = (1/\rho, \alpha) \f$ depending on \e select parameter.

  \param p : The vpPoint with updated \f$ (x, y) \f$ coordinates in the image plane that are used
  to create the vpFeatureVanishingPoint.

  \param select : Use either vpFeatureVanishingPoint::selectX() or vpFeatureVanishingPoint::selectY()
  to build \f$ {\bf s} = (x, y) \f$ visual feature, or use rather select
  vpFeatureVanishingPoint::selectOneOverRho() or vpFeatureVanishingPoint::selectAlpha() to build
  \f$ {\bf s} = (1/\rho, \alpha) \f$ visual feature.
*/
void vpFeatureBuilder::create(vpFeatureVanishingPoint &s, const vpPoint &p, unsigned int select)
{
  if ((vpFeatureVanishingPoint::selectX() & select) || (vpFeatureVanishingPoint::selectY() & select)) {
    s.set_x(p.get_x());
    s.set_y(p.get_y());
  }
  else if ((vpFeatureVanishingPoint::selectOneOverRho() & select) ||
          (vpFeatureVanishingPoint::selectAlpha() & select)) {
    double x = p.get_x();
    double y = p.get_y();

    s.setOneOverRho(1. / sqrt(x * x + y * y));
    s.setAlpha(atan2(y, x));
  }
}

/*!
  Initialize a vpFeatureVanishingPoint thanks to two vpFeatureLine.
  The vpFeatureVanishingPoint is initialized thanks to the coordinate of the
  intersection point in the image plan. All the parameters are given in meter.

  \param s : Visual feature to initialize; either \f$ {\bf s} = (x, y) \f$ or rather
  \f$ {\bf s} = (1/\rho, \alpha) \f$ depending on \e select parameter.

  \param L1 : The first vpFeatureLine.

  \param L2 : The second vpFeatureLine.

  \param select : Use either vpFeatureVanishingPoint::selectX() or vpFeatureVanishingPoint::selectY()
  to build \f$ {\bf s} = (x, y) \f$ visual feature, or use rather select
  vpFeatureVanishingPoint::selectOneOverRho() or vpFeatureVanishingPoint::selectAlpha() to build
  \f$ {\bf s} = (1/\rho, \alpha) \f$ visual feature.

  \warning An exception is thrown if the two lines are parallel when cartesian coordinates \f$ {\bf s} = (x, y) \f$ are
  used.
*/
void vpFeatureBuilder::create(vpFeatureVanishingPoint &s, const vpFeatureLine &L1, const vpFeatureLine &L2,
                              unsigned int select)
{
  if ((vpFeatureVanishingPoint::selectX() & select) || (vpFeatureVanishingPoint::selectY() & select)) {
    double rho_l = L1.getRho();
    double rho_r = L2.getRho();
    double theta_l = L1.getTheta();
    double theta_r = L2.getTheta();
    double c_l = cos(theta_l);
    double c_r = cos(theta_r);
    double s_l = sin(theta_l);
    double s_r = sin(theta_r);

    double min = 0.0001;
    if (fabs(theta_r - theta_l) < min || fabs(fabs(theta_r - theta_l) - M_PI) < min ||
        fabs(fabs(theta_r - theta_l) - 2 * M_PI) < min) {
      vpCERROR << "There is no vanishing point : the lines are parallel in the "
        "image plane"
        << std::endl;
      throw(vpFeatureException(vpFeatureException::badInitializationError,
                               "There is no vanishing point : the lines are "
                               "parallel in the image plane"));
    }

    double y = (rho_r * c_l - rho_l * c_r) / (-s_l * c_r + s_r * c_l);
    double x = (rho_r * s_l - rho_l * s_r) / (-c_l * s_r + c_r * s_l);

    s.set_x(x);
    s.set_y(y);
  }
  else if ((vpFeatureVanishingPoint::selectOneOverRho() & select) ||
          (vpFeatureVanishingPoint::selectAlpha() & select)) {
    double rho_1 = L1.getRho();
    double theta_1 = L1.getTheta();
    double rho_2 = L2.getRho();
    double theta_2 = L2.getTheta();

    double theta_diff = theta_1 - theta_2;

    double denom = sqrt(rho_1 * rho_1 + rho_2 * rho_2 - 2 * rho_1 * rho_2 * cos(theta_diff));
    double one_over_rho = sin(theta_diff) / denom;
    double alpha = atan2(rho_1 * cos(theta_2) - rho_2 * cos(theta_1), rho_2 * sin(theta_1) - rho_1 * sin(theta_2));

    s.setOneOverRho(one_over_rho);
    s.setAlpha(alpha);
  }
}

/*!
  Initialize a vpFeatureVanishingPoint thanks to two vpLine.
  The vpFeatureVanishingPoint is initialized thanks to the coordinate of the
  intersection point in the image plan. All the parameters are given in meter.

  \param s : Visual feature to initialize; either \f$ {\bf s} = (x, y) \f$ or rather
  \f$ {\bf s} = (1/\rho, \alpha) \f$ depending on \e select parameter.

  \param L1 : The first vpLine.

  \param L2 : The second vpLine.

  \param select : Use either vpFeatureVanishingPoint::selectX() or vpFeatureVanishingPoint::selectY()
  to build \f$ {\bf s} = (x, y) \f$ visual feature, or use rather select
  vpFeatureVanishingPoint::selectOneOverRho() or vpFeatureVanishingPoint::selectAlpha() to build
  \f$ {\bf s} = (1/\rho, \alpha) \f$ visual feature.

  \warning An exception is thrown if the two lines are parallel when cartesian coordinates \f$ {\bf s} = (x, y) \f$ are
  used.

*/
void vpFeatureBuilder::create(vpFeatureVanishingPoint &s, const vpLine &L1, const vpLine &L2, unsigned int select)
{
  vpFeatureLine l1, l2;
  vpFeatureBuilder::create(l1, L1);
  vpFeatureBuilder::create(l2, L2);

  vpFeatureBuilder::create(s, l1, l2, select);
}

/*!
  Initialize a vpFeatureVanishingPoint thanks to two vpLine.
  The vpFeatureVanishingPoint is initialized thanks to the coordinate of the
  intersection point in the image plan. All the parameters are given in meter.

  \param s : Visual feature to initialize; either \f$ {\bf s} = (x, y) \f$ or rather
  \f$ {\bf s} = (1/\rho, \alpha) \f$ depending on \e select parameter.

  \param cam : Camera parameters used to convert image point coordinates from pixel in meter in the image plane.
  \param line1_ip1, line1_ip2 : The first line defined by 2 image points with pixel coordinates in the image.
  \param line2_ip1, line2_ip2 : The second line defined by 2 image points with pixel coordinates in the image.

  \param select : Use either vpFeatureVanishingPoint::selectX() or vpFeatureVanishingPoint::selectY()
  to build \f$ {\bf s} = (x, y) \f$ visual feature, or use rather select
  vpFeatureVanishingPoint::selectOneOverRho() or vpFeatureVanishingPoint::selectAlpha() to build
  \f$ {\bf s} = (1/\rho, \alpha) \f$ visual feature.
*/
void vpFeatureBuilder::create(vpFeatureVanishingPoint &s, const vpCameraParameters &cam, const vpImagePoint &line1_ip1,
                              const vpImagePoint &line1_ip2, const vpImagePoint &line2_ip1,
                              const vpImagePoint &line2_ip2, unsigned int select)
{
  double x1 = 0, y1 = 0;
  double x2 = 0, y2 = 0;

  // First line
  vpPixelMeterConversion::convertPoint(cam, line1_ip1, x1, y1);
  vpPixelMeterConversion::convertPoint(cam, line1_ip2, x2, y2);

  double theta_1 = atan2(-(x1 - x2), y1 - y2);
  double rho_1 = ((x1 + x2) * cos(theta_1) + (y1 + y2) * sin(theta_1)) / 2.;

  // Second line
  vpPixelMeterConversion::convertPoint(cam, line2_ip1, x1, y1);
  vpPixelMeterConversion::convertPoint(cam, line2_ip2, x2, y2);

  double theta_2 = atan2(-(x1 - x2), y1 - y2);
  double rho_2 = ((x1 + x2) * cos(theta_2) + (y1 + y2) * sin(theta_2)) / 2.;

  if ((vpFeatureVanishingPoint::selectX() & select) || (vpFeatureVanishingPoint::selectY() & select)) {
    double min = 0.0001;
    double theta_diff = theta_1 - theta_2;

    if (fabs(theta_diff) < min || fabs(fabs(theta_diff) - M_PI) < min || fabs(fabs(theta_diff) - 2 * M_PI) < min) {
      throw(vpException(vpException::fatalError,
                        "There is no vanishing point : the lines are parallel in the image plane"));
    }

    double x = (sin(theta_1) * rho_2 - sin(theta_2) * rho_1) / sin(theta_diff);
    double y = (cos(theta_2) * rho_1 - cos(theta_1) * rho_2) / sin(theta_diff);

    s.set_x(x);
    s.set_y(y);
  }
  else if ((vpFeatureVanishingPoint::selectOneOverRho() & select)) {
    double theta_diff = theta_1 - theta_2;
    double denom = sqrt(rho_1 * rho_1 + rho_2 * rho_2 - 2 * rho_1 * rho_2 * cos(theta_diff));
    double one_over_rho = sin(theta_diff) / denom;
    //    if (one_over_rho < 0) {
    //      one_over_rho = -one_over_rho;
    //    }
    double alpha = atan2(rho_1 * cos(theta_2) - rho_2 * cos(theta_1), rho_2 * sin(theta_1) - rho_1 * sin(theta_2));

    s.setOneOverRho(one_over_rho);
    s.setAlpha(alpha);
  }
  else if ((vpFeatureVanishingPoint::selectAtanOneOverRho() & select)) {
    double theta_diff = theta_1 - theta_2;
    double denom = sqrt(rho_1 * rho_1 + rho_2 * rho_2 - 2 * rho_1 * rho_2 * cos(theta_diff));
    double alpha = atan2(rho_1 * cos(theta_2) - rho_2 * cos(theta_1), rho_2 * sin(theta_1) - rho_1 * sin(theta_2));
    double one_over_rho = sin(theta_diff) / denom;
    double atan_one_over_rho = atan2(sin(theta_diff), denom);

    //    if (one_over_rho < 0) {
    //      one_over_rho = -one_over_rho;
    //    }

    s.setOneOverRho(one_over_rho);
    s.setAtanOneOverRho(atan_one_over_rho);
    s.setAlpha(alpha);
  }
}
END_VISP_NAMESPACE
