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
 * Basic contours extraction based on the orignal work of
 * Sina Samangooei (ss@ecs.soton.ac.uk).
 *
 * Authors:
 * Souriya Trinh
 *
 *****************************************************************************/
/**
 * Copyright (c) 2011, The University of Southampton and the individual
 * contributors. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   *  Redistributions of source code must retain the above copyright notice,
 *  this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 *   * Neither the name of the University of Southampton nor the names of its
 *  contributors may be used to endorse or promote products derived from this
 *  software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*!
  \file vpContours.h
  \brief Basic contours extraction.
*/

#ifndef __vpContours_h__
#define __vpContours_h__

#include <visp3/core/vpColor.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpPolygon.h>

namespace
{
typedef enum {
  NORTH,
  NORTH_EAST,
  EAST,
  SOUTH_EAST,
  SOUTH,
  SOUTH_WEST,
  WEST,
  NORTH_WEST,
  LAST_DIRECTION
} vpDirectionType;

struct vpDirection {
  vpDirectionType m_direction;

  int m_dirx[8];
  int m_diry[8];

  vpDirection()
  {
    m_dirx[0] = 0;
    m_dirx[1] = 1;
    m_dirx[2] = 1;
    m_dirx[3] = 1;
    m_dirx[4] = 0;
    m_dirx[5] = -1;
    m_dirx[6] = -1;
    m_dirx[7] = -1;

    m_diry[0] = -1;
    m_diry[1] = -1;
    m_diry[2] = 0;
    m_diry[3] = 1;
    m_diry[4] = 1;
    m_diry[5] = 1;
    m_diry[6] = 0;
    m_diry[7] = -1;
  }

  vpDirection clockwise()
  {
    vpDirection direction;
    int directionSize = LAST_DIRECTION;
    direction.m_direction = vpDirectionType(((int)m_direction + 1) % directionSize);

    return direction;
  }

  vpDirection counterClockwise()
  {
    vpDirection direction;
    int directionSize = (int)LAST_DIRECTION;
    int idx = vpMath::modulo((int)m_direction - 1, directionSize);
    direction.m_direction = vpDirectionType(idx);

    return direction;
  }

  vpImagePoint active(const vpImage<int> &I, const vpImagePoint &point)
  {
    int yy = (int)(point.get_i() + m_diry[(int)m_direction]);
    int xx = (int)(point.get_j() + m_dirx[(int)m_direction]);

    if (xx < 0 || xx >= (int)I.getWidth() || yy < 0 || yy >= (int)I.getHeight()) {
      return vpImagePoint(-1, -1);
    }

    int pixel = I[yy][xx];
    return pixel != 0 ? vpImagePoint(yy, xx) : vpImagePoint(-1, -1);
  }
};
}

namespace vp
{
typedef enum {
  CONTOUR_OUTER, /*!< Outer contour. */
  CONTOUR_HOLE   /*!< Hole contour. */
} vpContourType;

typedef enum {
  CONTOUR_RETR_TREE,    /*!< Retrieve all the contours with the hierarchy stored
                           in a tree. */
  CONTOUR_RETR_LIST,    /*!< Retrieve all the contours without any hierarchy. */
  CONTOUR_RETR_EXTERNAL /*!< Retrieve only external contours. */
} vpContourRetrievalType;

struct vpContour {
  std::vector<vpContour *> m_children;
  vpContourType m_contourType;
  vpContour *m_parent;
  std::vector<vpImagePoint> m_points;

  vpContour() : m_children(), m_contourType(vp::CONTOUR_HOLE), m_parent(NULL), m_points() {}

  vpContour(const vpContourType &type) : m_children(), m_contourType(type), m_parent(NULL), m_points() {}

  vpContour(const vpContour &contour)
    : m_children(), m_contourType(contour.m_contourType), m_parent(NULL), m_points(contour.m_points)
  {

    // Copy the underlying contours
    for (std::vector<vpContour *>::const_iterator it = contour.m_children.begin(); it != contour.m_children.end();
         ++it) {
      vpContour *copy = new vpContour(**it);
      copy->m_parent = this;
      m_children.push_back(copy);
    }
  }

  ~vpContour()
  {
    for (std::vector<vpContour *>::iterator it = m_children.begin(); it != m_children.end(); ++it) {
      (*it)->m_parent = NULL;
      if (*it != NULL) {
        delete *it;
        *it = NULL;
      }
    }
  }

  vpContour &operator=(const vpContour &other)
  {
    m_contourType = other.m_contourType;

    if (m_parent == NULL) {
      // We are a root or an unintialized contour so delete everything
      for (std::vector<vpContour *>::iterator it = m_children.begin(); it != m_children.end(); ++it) {
        (*it)->m_parent = NULL;
        if (*it != NULL) {
          delete *it;
          *it = NULL;
        }
      }
    } else {
      // Make the current contour the root contour
      // to avoid problem when deleting
      m_parent = NULL;
    }

    m_children.clear();
    for (std::vector<vpContour *>::const_iterator it = other.m_children.begin(); it != other.m_children.end(); ++it) {
      vpContour *copy = new vpContour(**it);
      copy->m_parent = this;
      m_children.push_back(copy);
    }

    return *this;
  }

  void setParent(vpContour *parent)
  {
    m_parent = parent;

    if (parent != NULL) {
      parent->m_children.push_back(this);
    }
  }
};

VISP_EXPORT void drawContours(vpImage<unsigned char> &I, const std::vector<std::vector<vpImagePoint> > &contours,
                              unsigned char grayValue = 255);
VISP_EXPORT void drawContours(vpImage<vpRGBa> &I, const std::vector<std::vector<vpImagePoint> > &contours,
                              const vpColor &color);

VISP_EXPORT void findContours(const vpImage<unsigned char> &I_original, vpContour &contours,
                              std::vector<std::vector<vpImagePoint> > &contourPts,
                              const vpContourRetrievalType &retrievalMode = vp::CONTOUR_RETR_TREE);
}

#endif
