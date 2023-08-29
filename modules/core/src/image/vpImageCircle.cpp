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
 * Image circle, i.e. circle in the image space.
 *
*****************************************************************************/

#include <visp3/core/vpImageCircle.h>

vpImageCircle::vpImageCircle()
  : m_center()
  , m_radius(0.)
{

}

vpImageCircle::vpImageCircle(const vpImagePoint &center, const float &radius)
  : m_center(center)
  , m_radius(radius)
{

}

#ifdef HAVE_OPENCV_CORE
vpImageCircle::vpImageCircle(const cv::Vec3f &vec)
  : m_center(vec[1], vec[0])
  , m_radius(vec[2])
{

}
#endif

vpImageCircle::~vpImageCircle()
{

}

vpImagePoint vpImageCircle::getCenter() const
{
  return m_center;
}

float vpImageCircle::getRadius() const
{
  return m_radius;
}

vpRect vpImageCircle::getBBox() const
{
  vpRect bbox(m_center - vpImagePoint(m_radius, m_radius), 2 * m_radius, 2 * m_radius);
  return bbox;
}

float vpImageCircle::get_n20() const
{
  return m_radius * m_radius / 4;
}

float vpImageCircle::get_n02() const
{
  return m_radius * m_radius / 4;
}

float vpImageCircle::get_n11() const
{
  return 0.;
};
