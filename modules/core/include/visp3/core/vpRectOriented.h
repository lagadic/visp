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
 * Defines a (possibly oriented) rectangle in the plane.
 *
 * Author:
 * Pierre Chatelain
 * Marc Pouliquen
 *
 *****************************************************************************/

#ifndef vpRectOriented_h
#define vpRectOriented_h

/*!
  \class vpRectOriented
  \ingroup group_core_geometry
  \brief Defines an oriented rectangle in the plane.
*/

#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpRect.h>

class VISP_EXPORT vpRectOriented
{
public:
  vpRectOriented();

  vpRectOriented(const vpImagePoint &center, const double width, const double height, const double theta = 0);

  vpRectOriented(const vpRect &rect);

  vpRectOriented &operator=(const vpRectOriented &rect);

  vpRectOriented &operator=(const vpRect &rect);

  operator vpRect();

  void setCenter(const vpImagePoint &center);

  void setPoints(const vpImagePoint &topLeft, const vpImagePoint &topRight, const vpImagePoint &bottomLeft,
                 const vpImagePoint &bottomRight);

  vpImagePoint getCenter() const;

  vpImagePoint getTopLeft() const;

  vpImagePoint getTopRight() const;

  vpImagePoint getBottomLeft() const;

  vpImagePoint getBottomRight() const;

  void setSize(double width, double height);

  double getWidth() const;

  double getHeight() const;

  void setOrientation(double theta);

  double getOrientation() const;

  bool isInside(const vpImagePoint &point) const;

private:
  vpImagePoint m_center;
  double m_width;
  double m_height;
  double m_theta;
  vpImagePoint m_topLeft;
  vpImagePoint m_topRight;
  vpImagePoint m_bottomLeft;
  vpImagePoint m_bottomRight;
  bool isLeft(const vpImagePoint &pointToTest, const vpImagePoint &point1, const vpImagePoint &point2) const;
};
#endif // vpRectOriented_h
