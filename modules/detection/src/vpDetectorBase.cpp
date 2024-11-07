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
 * Base class for object detection.
 *
*****************************************************************************/
#include <visp3/core/vpConfig.h>

#include <visp3/detection/vpDetectorBase.h>
BEGIN_VISP_NAMESPACE

/*!
  Default constructor.
*/
vpDetectorBase::vpDetectorBase() : m_polygon(), m_message(), m_nb_objects(0), m_timeout_ms(0) { }

vpDetectorBase::vpDetectorBase(const vpDetectorBase &o)
  : m_polygon(o.m_polygon), m_message(o.m_message), m_nb_objects(o.m_nb_objects), m_timeout_ms(o.m_timeout_ms)
{ }

/*!
        Returns ith object container box as a vector of points.
*/
std::vector<vpImagePoint> &vpDetectorBase::getPolygon(size_t i)
{
  if (i < m_polygon.size()) {
    return m_polygon[i];
  }
  else {
    throw(vpException(vpException::badValue, "Bad index to retrieve object %d. Only %d objects are detected.", i,
                      m_polygon.size()));
  }
}

/*!
        Returns the contained message of the ith object if there is one.
*/
std::string &vpDetectorBase::getMessage(size_t i)
{
  if (i < m_polygon.size()) {
    return m_message[i];
  }
  else {
    throw(vpException(vpException::badValue, "Bad index to retrieve object %d . Only %d objects are detected.", i,
                      m_polygon.size()));
  }
}

/*!
        Return the center of gravity location of the ith object.
*/
vpImagePoint vpDetectorBase::getCog(size_t i) const
{
  vpImagePoint cog(0, 0);
  size_t m_polygon_i_size = m_polygon[i].size();
  for (size_t j = 0; j < m_polygon_i_size; ++j) {
    cog += m_polygon[i][j];
  }
  cog /= static_cast<double>(m_polygon[i].size());
  return cog;
}

/*!
Return the bounding box of the ith object.
*/
vpRect vpDetectorBase::getBBox(size_t i) const
{
  assert(m_polygon[i].size() > 2);

  double left, right;
  double top, bottom;
  left = m_polygon[i][0].get_u();
  right = m_polygon[i][0].get_u();
  top = m_polygon[i][0].get_v();
  bottom = m_polygon[i][0].get_v();
  size_t m_polygon_i_size = m_polygon[i].size();
  for (size_t j = 0; j < m_polygon_i_size; ++j) {
    double u = m_polygon[i][j].get_u();
    double v = m_polygon[i][j].get_v();
    if (u < left) {
      left = u;
    }
    if (u > right) {
      right = u;
    }
    if (v < top) {
      top = v;
    }
    if (v > bottom) {
      bottom = v;
    }
  }
  vpRect roi(vpImagePoint(top, left), vpImagePoint(bottom, right));
  return roi;
}

END_VISP_NAMESPACE
