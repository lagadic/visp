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
 * Defines a 2D triangle.
 *
 * Author:
 * Amaury Dame
 * Nicolas Melchior
 *
 *****************************************************************************/

#include <visp3/core/vpDebug.h>
#include <visp3/core/vpTriangle.h>

/*!
  Basic constructor.

  By default, the three 2D points coordinates which define the triangle are
  \f$ (0,0) \f$, \f$ (1,0) \f$ and \f$ (0,1) \f$.
*/
vpTriangle::vpTriangle()
  : goodTriange(true), S1(), uvinv00(0), uvinv01(0), uvinv10(0), uvinv11(0), ptempo0(0), ptempo1(0), area(0), apex1(),
    apex2(), apex3()
{
  init(vpImagePoint(0, 0), vpImagePoint(1, 0), vpImagePoint(0, 1));
}

/*!
  Constructor which initialise the triangle thanks to the three 2D points \f$
  iP1 \f$, \f$ iP2 \f$ and \f$ iP3 \f$

  \param iP1 : The first apex of the triangle.
  \param iP2 : The first apex of the triangle.
  \param iP3 : The first apex of the triangle.
*/
vpTriangle::vpTriangle(const vpImagePoint &iP1, const vpImagePoint &iP2, const vpImagePoint &iP3)
  : goodTriange(true), S1(), uvinv00(0), uvinv01(0), uvinv10(0), uvinv11(0), ptempo0(0), ptempo1(0), area(0), apex1(),
    apex2(), apex3()
{
  init(iP1, iP2, iP3);
}

/*!
  Copy constructor

  \param tri : The triangle used for the initialisation.
*/
vpTriangle::vpTriangle(const vpTriangle &tri)
  : goodTriange(true), S1(), uvinv00(0), uvinv01(0), uvinv10(0), uvinv11(0), ptempo0(0), ptempo1(0), area(0), apex1(),
    apex2(), apex3()
{
  *this = tri;
}

/*!
  Basic destructor
*/
vpTriangle::~vpTriangle() {}

/*!
  Assign \e tri to this triangle and return a reference to this triangle.
*/
vpTriangle &vpTriangle::operator=(const vpTriangle &tri)
{
  goodTriange = tri.goodTriange;
  S1 = tri.S1;
  uvinv00 = tri.uvinv00;
  uvinv01 = tri.uvinv01;
  uvinv10 = tri.uvinv10;
  uvinv11 = tri.uvinv11;
  ptempo0 = tri.ptempo0;
  ptempo1 = tri.ptempo1;
  area = tri.area;
  apex1 = tri.apex1;
  apex2 = tri.apex2;
  apex3 = tri.apex3;
  return *this;
};

/*!
  Initialise the triangle thanks to the three 2D points \f$ iP1 \f$, \f$ iP2
  \f$ and \f$ iP3 \f$

  \param iP1 : The first apex of the triangle.
  \param iP2 : The first apex of the triangle.
  \param iP3 : The first apex of the triangle.
*/
void vpTriangle::buildFrom(const vpImagePoint &iP1, const vpImagePoint &iP2, const vpImagePoint &iP3)
{
  init(iP1, iP2, iP3);
}

void vpTriangle::init(const vpImagePoint &iP1, const vpImagePoint &iP2, const vpImagePoint &iP3)
{
  ptempo0 = ptempo1 = 0.;
  apex1 = iP1;
  apex2 = iP2;
  apex3 = iP3;

  vpMatrix uv(2, 2);
  vpMatrix uvinv(2, 2);

  uv[0][0] = iP2.get_i() - iP1.get_i();
  uv[1][0] = iP3.get_i() - iP1.get_i();
  uv[0][1] = iP2.get_j() - iP1.get_j();
  uv[1][1] = iP3.get_j() - iP1.get_j();
  try {
    uvinv = uv.inverseByLU();
    goodTriange = true;
  } catch (...) {
    goodTriange = false;
    std::cout << "Empty triangle" << std::endl;
  }

  uvinv00 = uvinv[0][0];
  uvinv01 = uvinv[0][1];
  uvinv10 = uvinv[1][0];
  uvinv11 = uvinv[1][1];
  S1 = iP1;
  area = 0.5 * fabs(uv.det());
}

/*!
  Check if the 2D point \f$ iP \f$ is inside the triangle.

  \param iP : The point which coulb be inside the triangle.
  \param threshold : A threshold used to define the accuracy of the
  computation when the point is very near from the edges of the triangle. 0 is
  the smallest value.

  \return Returns true if the point is inside the triangle. Returns false
  otherwise.
*/
bool vpTriangle::inTriangle(const vpImagePoint &iP, double threshold)
{
  if (!goodTriange)
    return false;

  if (threshold < 0)
    threshold = 0;

  ptempo0 = iP.get_i() - S1.get_i();
  ptempo1 = iP.get_j() - S1.get_j();

  double p_ds_uv0 = ptempo0 * uvinv00 + ptempo1 * uvinv10;
  double p_ds_uv1 = ptempo0 * uvinv01 + ptempo1 * uvinv11;

  return (p_ds_uv0 + p_ds_uv1 < 1. + threshold && p_ds_uv0 > -threshold && p_ds_uv1 > -threshold);
}
