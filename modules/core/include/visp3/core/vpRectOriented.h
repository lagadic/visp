/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
 * Defines a (possibly oriented) rectangle in the plane.
 */

#ifndef VP_RECT_ORIENTED_H
#define VP_RECT_ORIENTED_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpRect.h>

BEGIN_VISP_NAMESPACE
/*!
 * \class vpRectOriented
 * \ingroup group_core_geometry
 * \brief Defines an oriented rectangle in the plane.
*/
class VISP_EXPORT vpRectOriented
{
public:
  vpRectOriented();
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  vpRectOriented(const vpRectOriented &) = default;
#else
  vpRectOriented(const vpRectOriented &rect);
#endif

  vpRectOriented(const vpImagePoint &center, double width, double height, double theta = 0);

  VP_EXPLICIT vpRectOriented(const vpRect &rect);

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  vpRectOriented &operator=(const vpRectOriented &) = default;
#else
  vpRectOriented &operator=(const vpRectOriented &rect);
#endif

  vpRectOriented &operator=(const vpRect &rect);

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  VP_EXPLICIT operator vpRect();
#endif

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
END_VISP_NAMESPACE
#endif // _vpRectOriented_h_
