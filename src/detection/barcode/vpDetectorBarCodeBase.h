/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2014 by INRIA. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 *
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * Base class for bar code detection.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#ifndef __vpDetectorBarCodeBase_h__
#define __vpDetectorBarCodeBase_h__

#include <vector>
#include <utility>
#include <string>
#include <assert.h>

#include <visp/vpImage.h>
#include <visp/vpImagePoint.h>
#include <visp/vpRect.h>

/*!
  \class vpDetectorBarCodeBase

  Base class for bar code detector.

  The example given in tutorial-barcode-detector.cpp shows how to detect one or more bar codes in an
  image. In tutorial-barcode-detector-live.cpp you will find an other example that shows how
  to use this class to detect bar codes in images acquired by a camera.
 */
class VISP_EXPORT vpDetectorBarCodeBase
{
protected:
  std::vector< std::vector<vpImagePoint> > m_polygon;
  std::vector< std::string > m_message;
  size_t m_nb_objects;

public:
  /*!
     Default constructor.
   */
  vpDetectorBarCodeBase() : m_polygon(), m_message(), m_nb_objects(0) {}
  /*!
     Default destructor.
     */
  virtual ~vpDetectorBarCodeBase() {};

  /*!
    Detect bar code pattern in image.
    \param I : Image where to detect pattern.
    \return true if one or multiple codes are detected, false otherwise.
   */
  virtual bool detect(const vpImage<unsigned char> &I) = 0;
  /*!
    Returns pattern container box as a vector of points.
   */
  std::vector< std::vector<vpImagePoint> > & getPolygon()
  {
    return m_polygon;
  }

  /*!
    Returns ith pattern container box as a vector of points.
   */
  std::vector<vpImagePoint> & getPolygon(size_t i)
  {
    if (i < m_polygon.size())
      return m_polygon[i];
    else
      throw(vpException(vpException::badValue, "Bad index %d to retrieve bar code. Only %d codes are detected.", i, m_polygon.size()));
  }
  /*!
    Returns the contained message of the ith bar code if there is one.
   */
  std::string & getMessage(size_t i)
  {
    if (i < m_polygon.size())
      return m_message[i];
    else
      throw(vpException(vpException::badValue, "Bad index %d to retrieve bar code. Only %d codes are detected.", i, m_polygon.size()));
  }
  /*!
    Returns the contained message of the ith bar code if there is one.
   */
  std::vector< std::string > & getMessage()
  {
    return m_message;
  }
  /*!
    Return the number of objects that are detected.
    */
  size_t getNbObjects() const {return m_nb_objects; }
  /*!
    Return the center of gravity location of the ith bar code.
   */
  vpImagePoint getCog(size_t i) const
  {
    vpImagePoint cog(0,0);
    for(size_t j=0; j < m_polygon[i].size(); j++) {
      cog += m_polygon[i][j];
    }
    cog /= m_polygon[i].size();
    return cog;
  }
  /*!
    Return the bounding box of the ith bar code.
   */
  vpRect getBBox(size_t i) const
  {
    assert(m_polygon[i].size() > 2);

    double left, right;
    double top, bottom;
    left = right = m_polygon[i][0].get_u();
    top = bottom = m_polygon[i][0].get_v();
    for(size_t j=0; j < m_polygon[i].size(); j++) {
      double u = m_polygon[i][j].get_u();
      double v = m_polygon[i][j].get_v();
      if (u < left) left = u;
      if (u > right) right = u;
      if (v < top) top = v;
      if (v > bottom) bottom = v;
    }
    vpRect roi(vpImagePoint(top, left), vpImagePoint(bottom, right));
    return roi;
  }
};

#endif
