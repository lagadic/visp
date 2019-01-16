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
  \file vpContours.cpp
  \brief Basic contours extraction.
*/

#include <map>
#include <visp3/imgproc/vpImgproc.h>

namespace
{
bool fromTo(const vpImagePoint &from, const vpImagePoint &to, vpDirection &direction)
{
  if (from == to) {
    return false;
  }

  if (std::fabs(from.get_i() - to.get_i()) < std::numeric_limits<double>::epsilon()) {
    if (from.get_j() < to.get_j()) {
      direction.m_direction = EAST;
    } else {
      direction.m_direction = WEST;
    }
  } else if (from.get_i() < to.get_i()) {
    if (std::fabs(from.get_j() - to.get_j()) < std::numeric_limits<double>::epsilon()) {
      direction.m_direction = SOUTH;
    } else if (from.get_j() < to.get_j()) {
      direction.m_direction = SOUTH_EAST;
    } else {
      direction.m_direction = SOUTH_WEST;
    }
  } else {
    if (std::fabs(from.get_j() - to.get_j()) < std::numeric_limits<double>::epsilon()) {
      direction.m_direction = NORTH;
    } else if (from.get_j() < to.get_j()) {
      direction.m_direction = NORTH_EAST;
    } else {
      direction.m_direction = NORTH_WEST;
    }
  }

  return true;
}

bool crossesEastBorder(const vpImage<int> &I, bool checked[8], const vpImagePoint &point)
{
  vpDirection direction;
  if (!fromTo(point, vpImagePoint(point.get_i(), point.get_j() + 1), direction)) {
    return false;
  }

  bool b = checked[(int)direction.m_direction];

  if (point.get_i() < 0 || point.get_j() < 0) {
    return false;
  }

  unsigned int i = (unsigned int)point.get_i();
  unsigned int j = (unsigned int)point.get_j();

  return I[i][j] != 0 && ((unsigned int)point.get_j() == I.getWidth() - 1 || b);
}

void addContourPoint(vpImage<int> &I, vp::vpContour *border, const vpImagePoint &point, bool checked[8], const int nbd)
{
  border->m_points.push_back(vpImagePoint(point.get_i() - 1, point.get_j() - 1)); // remove 1-pixel padding

  unsigned int i = (unsigned int)point.get_i();
  unsigned int j = (unsigned int)point.get_j();

  if (crossesEastBorder(I, checked, point)) {
    I[i][j] = -nbd;
  } else if (I[i][j] == 1) {
    // Only set if the pixel has not been visited before (3.4) (b)
    I[i][j] = nbd;
  } // Otherwise leave it alone
}

void followBorder(vpImage<int> &I, const vpImagePoint &ij, vpImagePoint &i2j2, vp::vpContour *border, const int nbd)
{
  vpDirection dir;
  if (!fromTo(ij, i2j2, dir)) {
    throw vpException(vpException::fatalError, "ij == i2j2");
  }

  vpDirection trace = dir.clockwise();
  vpImagePoint i1j1(-1, -1);

  // Find i1j1 (3.1)
  while (trace.m_direction != dir.m_direction) {
    vpImagePoint activePixel = trace.active(I, ij);

    if (activePixel.get_i() >= 0 && activePixel.get_j() >= 0) {
      i1j1 = activePixel;
      break;
    }

    trace = trace.clockwise();
  }

  if (i1j1.get_i() < 0 || i1j1.get_j() < 0) {
    //(3.1) ; single pixel contour
    return;
  }

  i2j2 = i1j1;
  vpImagePoint i3j3 = ij; //(3.2)

  bool checked[8] = {false, false, false, false, false, false, false, false};

  while (true) {
    if (!fromTo(i3j3, i2j2, dir)) {
      throw vpException(vpException::fatalError, "i3j3 == i2j2");
    }

    trace = dir.counterClockwise();
    vpImagePoint i4j4(-1, -1);

    // Reset checked
    for (int cpt = 0; cpt < 8; cpt++) {
      checked[cpt] = false;
    }

    while (true) {
      i4j4 = trace.active(I, i3j3); //(3.3)
      if (i4j4.get_i() >= 0 && i4j4.get_j() >= 0) {
        break;
      }

      checked[(int)trace.m_direction] = true;
      trace = trace.counterClockwise();
    }

    addContourPoint(I, border, i3j3, checked, nbd);

    if (i4j4 == ij && i3j3 == i1j1) {
      //(3.5)
      break;
    }

    //(3.5)
    i2j2 = i3j3;
    i3j3 = i4j4;
  }
}

bool isOuterBorderStart(const vpImage<int> &I, unsigned int i, unsigned int j)
{
  return (I[i][j] == 1 && (j == 0 || I[i][j - 1] == 0));
}

bool isHoleBorderStart(const vpImage<int> &I, unsigned int i, unsigned int j)
{
  return (I[i][j] >= 1 && (j == I.getWidth() - 1 || I[i][j + 1] == 0));
}

void getContoursList(const vp::vpContour &root, const int level, vp::vpContour &contour_list)
{
  if (level > 0) {
    vp::vpContour *contour_node = new vp::vpContour;
    contour_node->m_contourType = root.m_contourType;
    contour_node->m_points = root.m_points;

    contour_list.m_children.push_back(contour_node);
  }

  for (std::vector<vp::vpContour *>::const_iterator it = root.m_children.begin(); it != root.m_children.end(); ++it) {
    getContoursList(**it, level + 1, contour_list);
  }
}
} // namespace

/*!
  \ingroup group_imgproc_contours

  Draw the input contours on the binary image.

  \param I : Grayscale image where we want to draw the input contours.
  \param contours : Detected contours.
  \param grayValue : Drawing grayscale color.
*/
void vp::drawContours(vpImage<unsigned char> &I, const std::vector<std::vector<vpImagePoint> > &contours,
                      unsigned char grayValue)
{
  if (I.getSize() == 0) {
    return;
  }

  for (std::vector<std::vector<vpImagePoint> >::const_iterator it1 = contours.begin(); it1 != contours.end(); ++it1) {
    for (std::vector<vpImagePoint>::const_iterator it2 = it1->begin(); it2 != it1->end(); ++it2) {
      unsigned int i = (unsigned int)it2->get_i();
      unsigned int j = (unsigned int)it2->get_j();
      I[i][j] = grayValue;
    }
  }
}

/*!
  \ingroup group_imgproc_contours

  Draw the input contours on the color image.

  \param I : Color image where we want to draw the input contours.
  \param contours : Detected contours.
  \param color : Drawing color.
*/
void vp::drawContours(vpImage<vpRGBa> &I, const std::vector<std::vector<vpImagePoint> > &contours, const vpColor &color)
{
  if (I.getSize() == 0) {
    return;
  }

  for (std::vector<std::vector<vpImagePoint> >::const_iterator it1 = contours.begin(); it1 != contours.end(); ++it1) {
    for (std::vector<vpImagePoint>::const_iterator it2 = it1->begin(); it2 != it1->end(); ++it2) {
      unsigned int i = (unsigned int)it2->get_i();
      unsigned int j = (unsigned int)it2->get_j();
      I[i][j] = vpRGBa(color.R, color.G, color.B);
    }
  }
}

/*!
  \ingroup group_imgproc_contours

  Extract contours from a binary image.

  \param I_original : Input binary image (0 means background, 1 means
  foreground, other values are not allowed). \param contours : Detected
  contours. \param contourPts : List of contours, each contour contains a list
  of contour points. \param retrievalMode : Contour retrieval mode.
*/
void vp::findContours(const vpImage<unsigned char> &I_original, vpContour &contours,
                      std::vector<std::vector<vpImagePoint> > &contourPts, const vpContourRetrievalType &retrievalMode)
{
  if (I_original.getSize() == 0) {
    return;
  }

  // Clear output results
  contourPts.clear();

  // Copy uchar I_original into int I + padding
  vpImage<int> I(I_original.getHeight() + 2, I_original.getWidth() + 2);
  for (unsigned int i = 0; i < I.getHeight(); i++) {
    if (i == 0 || i == I.getHeight() - 1) {
      memset(I.bitmap, 0, sizeof(int) * I.getWidth());
    } else {
      I[i][0] = 0;
      for (unsigned int j = 0; j < I_original.getWidth(); j++) {
        I[i][j + 1] = I_original[i - 1][j];
      }
      I[i][I.getWidth() - 1] = 0;
    }
  }

  // Ref: http://openimaj.org/
  // Ref: Satoshi Suzuki and others. Topological structural analysis of
  // digitized binary images by border following.
  int nbd = 1;  // Newest border
  int lnbd = 1; // Last newest border

  // Background contour
  // By default the root contour is a hole contour
  vpContour *root = new vpContour(vp::CONTOUR_HOLE);

  std::map<int, vpContour *> borderMap;
  borderMap[lnbd] = root;

  for (unsigned int i = 0; i < I.getHeight(); i++) {
    lnbd = 1; // Reset LNBD at the beginning of each scan row

    for (unsigned int j = 0; j < I.getWidth(); j++) {
      int fji = I[i][j];

      bool isOuter = isOuterBorderStart(I, i, j);
      bool isHole = isHoleBorderStart(I, i, j);

      if (isOuter || isHole) { // else (1) (c)
        vpContour *border = new vpContour;
        vpContour *borderPrime = NULL;
        vpImagePoint from(i, j);

        if (isOuter) {
          //(1) (a)
          nbd++;
          from.set_j(from.get_j() - 1);
          border->m_contourType = vp::CONTOUR_OUTER;
          borderPrime = borderMap[lnbd];

          // Table 1
          switch (borderPrime->m_contourType) {
          case vp::CONTOUR_OUTER:
            border->setParent(borderPrime->m_parent);
            break;

          case vp::CONTOUR_HOLE:
            border->setParent(borderPrime);
            break;

          default:
            break;
          }
        } else {
          //(1) (b)
          nbd++;

          if (fji > 1) {
            lnbd = fji;
          }

          borderPrime = borderMap[lnbd];
          from.set_j(from.get_j() + 1);
          border->m_contourType = vp::CONTOUR_HOLE;

          // Table 1
          switch (borderPrime->m_contourType) {
          case vp::CONTOUR_OUTER:
            border->setParent(borderPrime);
            break;

          case vp::CONTOUR_HOLE:
            border->setParent(borderPrime->m_parent);
            break;

          default:
            break;
          }
        }

        vpImagePoint ij(i, j);
        followBorder(I, ij, from, border, nbd);

        //(3) (1) ; single pixel contour
        if (border->m_points.empty()) {
          border->m_points.push_back(vpImagePoint(ij.get_i() - 1, ij.get_j() - 1)); // remove 1-pixel padding
          I[i][j] = -nbd;
        }

        if (retrievalMode == CONTOUR_RETR_LIST || retrievalMode == CONTOUR_RETR_TREE) {
          // Add contour points
          contourPts.push_back(border->m_points);
        }

        borderMap[nbd] = border;
      }

      //(4)
      if (fji != 0 && fji != 1) {
        lnbd = std::abs(fji);
      }
    }
  }

  if (retrievalMode == CONTOUR_RETR_EXTERNAL || retrievalMode == CONTOUR_RETR_LIST) {
    // Delete contours content
    contours.m_parent = NULL;

    for (std::vector<vpContour *>::iterator it = contours.m_children.begin(); it != contours.m_children.end(); ++it) {
      (*it)->m_parent = NULL;
      if (*it != NULL) {
        delete *it;
        *it = NULL;
      }
    }

    contours.m_children.clear();
  }

  if (retrievalMode == CONTOUR_RETR_EXTERNAL) {
    // Add only external contours
    for (std::vector<vpContour *>::const_iterator it = root->m_children.begin(); it != root->m_children.end(); ++it) {
      // Save children
      std::vector<vpContour *> children_copy = (*it)->m_children;
      // Erase children
      (*it)->m_children.clear();
      // Copy contour
      contours.m_children.push_back(new vpContour(**it));
      // Restore children
      (*it)->m_children = children_copy;
      // Set parent to children
      for (size_t i = 0; i < contours.m_children.size(); i++) {
        contours.m_children[i]->m_parent = &contours;
      }
      contourPts.push_back((*it)->m_points);
    }
  } else if (retrievalMode == CONTOUR_RETR_LIST) {
    getContoursList(*root, 0, contours);

    // Set parent to root
    for (std::vector<vpContour *>::iterator it = contours.m_children.begin(); it != contours.m_children.end(); ++it) {
      (*it)->m_parent = &contours;
    }
  } else {
    // CONTOUR_RETR_TREE
    contours = *root;
  }

  delete root;
  root = NULL;
}
