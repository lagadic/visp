/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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

#ifndef vpTriangle_h
#define vpTriangle_h

/*!
  \class vpTriangle
  \ingroup group_core_geometry
  \brief Defines a 2D triangle.

  A triangle is internally represented by three 2D points.

  By default the three coordinates in the \f$ (i,j) \f$ frame (see
  vpImagePoint class documentation for more details about the frame.) are \f$
  (0,0) \f$, \f$ (1,0) \f$ and \f$ (0,1) \f$.
*/

#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpMatrix.h>

class VISP_EXPORT vpTriangle
{
private:
  bool goodTriange;
  vpImagePoint S1;
  double uvinv00;
  double uvinv01;
  double uvinv10;
  double uvinv11;
  double ptempo0;
  double ptempo1;
  double area;
  vpImagePoint apex1;
  vpImagePoint apex2;
  vpImagePoint apex3;

public:
  vpTriangle();
  vpTriangle(const vpTriangle &tri);
  virtual ~vpTriangle();

  vpTriangle &operator=(const vpTriangle &tri);

  vpTriangle(const vpImagePoint &iP1, const vpImagePoint &iP2, const vpImagePoint &iP3);

  void buildFrom(const vpImagePoint &iP1, const vpImagePoint &iP2, const vpImagePoint &iP3);

  bool inTriangle(const vpImagePoint &iP, double threshold = 0.00001);

  /*!
    Get the apexes of the triangle.

    \param iP1 : first apex.
    \param iP2 : second apex.
    \param iP3 : third apex.
  */
  void getTriangleApexes(vpImagePoint &iP1, vpImagePoint &iP2, vpImagePoint &iP3) const
  {
    iP1 = apex1;
    iP2 = apex2;
    iP3 = apex3;
  }

  /*!
    Return the area of the triangle.
    The area is computed when the triangle is built from three image points.

    \return The area of the triangle.
  */
  inline double getArea() const { return this->area; }

private:
  void init(const vpImagePoint &iP1, const vpImagePoint &iP2, const vpImagePoint &iP3);
};

#endif
