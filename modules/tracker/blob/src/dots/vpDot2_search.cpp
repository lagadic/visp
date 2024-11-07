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
 * Search a dot in an area.
 */

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/blob/vpDot2.h>

BEGIN_VISP_NAMESPACE

/**
 * \brief Performs the research of dots in the area when the point u, v is a good germ.
 *
 * \param data The data required for the algorithm.
 */
  void vpDot2::searchDotsAreaGoodGerm(vpSearchDotsInAreaGoodGermData &data)
{
// Compute the right border position for this possible germ
  unsigned int border_u;
  unsigned int border_v;
  bool good_germ = true;
  std::list<vpDot2>::iterator itbad;
  std::list<vpDot2>::iterator itnice;
  vpDot2 *dotToTest = nullptr;
  vpDot2 tmpDot;
  vpImagePoint cogTmpDot;
  if (findFirstBorder(data.m_I, data.m_u, data.m_v, border_u, border_v) == false) {
    // germ is not good.
    // Jump all the pixels between v,u and v,
    // dotToTest->getFirstBorder_u()
    data.m_u = border_u;
    data.m_v = border_v;
  }
  else {
    itbad = data.m_badDotsVector.begin();
    vpImagePoint cogBadDot;

    while ((itbad != data.m_badDotsVector.end()) && (good_germ == true)) {
      if ((static_cast<double>(data.m_u) >= (*itbad).m_bbox_u_min) && (static_cast<double>(data.m_u) <= (*itbad).m_bbox_u_max) &&
          (static_cast<double>(data.m_v) >= (*itbad).m_bbox_v_min) && (static_cast<double>(data.m_v) <= (*itbad).m_bbox_v_max)) {
        std::list<vpImagePoint>::const_iterator it_edges = m_ip_edges_list.begin();
        while ((it_edges != m_ip_edges_list.end()) && (good_germ == true)) {
          // Test if the germ belong to a previously detected dot:
          // - from the germ go right to the border and compare this
          //   position to the list of pixels of previously detected dots
          cogBadDot = *it_edges;
          if ((std::fabs(border_u - cogBadDot.get_u()) <=
               (vpMath::maximum(std::fabs(static_cast<double>(border_u)), std::fabs(cogBadDot.get_u())) *
                std::numeric_limits<double>::epsilon())) &&
              (std::fabs(data.m_v - cogBadDot.get_v()) <=
               (vpMath::maximum(std::fabs(static_cast<double>(data.m_v)), std::fabs(cogBadDot.get_v())) *
                std::numeric_limits<double>::epsilon()))) {
            good_germ = false;
          }
          ++it_edges;
        }
      }
      ++itbad;
    }

    if (!good_germ) {
      // Jump all the pixels between v,u and v,
      // dotToTest->getFirstBorder_u()
      data.m_u = border_u;
      data.m_v = border_v;
    }
    else {
      vpImagePoint germ;
      germ.set_u(data.m_u);
      germ.set_v(data.m_v);

      // otherwise estimate the width, height and surface of the dot we
      // created, and test it.
      if (dotToTest != nullptr) {
        delete dotToTest;
      }
      dotToTest = getInstance();
      dotToTest->setCog(germ);
      dotToTest->setGrayLevelMin(getGrayLevelMin());
      dotToTest->setGrayLevelMax(getGrayLevelMax());
      dotToTest->setGrayLevelPrecision(getGrayLevelPrecision());
      dotToTest->setSizePrecision(getSizePrecision());
      dotToTest->setGraphics(m_graphics);
      dotToTest->setGraphicsThickness(m_thickness);
      dotToTest->setComputeMoments(true);
      dotToTest->setArea(m_area);
      dotToTest->setEllipsoidShapePrecision(m_ellipsoidShapePrecision);
      dotToTest->setEllipsoidBadPointsPercentage(m_allowedBadPointsPercentage);

      // first compute the parameters of the dot.
      // if for some reasons this caused an error tracking
      // (dot partially out of the image...), check the next intersection
      if (dotToTest->computeParameters(data.m_I) == false) {
        // Jump all the pixels between v,u and v,
        // dotToTest->getFirstBorder_u()
        data.m_u = border_u;
        data.m_v = border_v;
      }
      else {
      // if the dot to test is valid,
        if (dotToTest->isValid(data.m_I, *this)) {
          vpImagePoint cogDotToTest = dotToTest->getCog();
          // Compute the distance to the center. The center used here is not the
          // area center available by area.getCenter(area_center_u,
          // area_center_v) but the center of the input area which may be
          // partially outside the image.

          double area_center_u = (data.m_area.getLeft() + (data.m_area.getWidth() / 2.0)) - 0.5;
          double area_center_v = (data.m_area.getTop() + (data.m_area.getHeight() / 2.0)) - 0.5;

          double thisDiff_u = cogDotToTest.get_u() - area_center_u;
          double thisDiff_v = cogDotToTest.get_v() - area_center_v;
          double thisDist = sqrt((thisDiff_u * thisDiff_u) + (thisDiff_v * thisDiff_v));

          bool stopLoop = false;
          itnice = data.m_niceDots.begin();

          while ((itnice != data.m_niceDots.end()) && (stopLoop == false)) {
            tmpDot = *itnice;

            // --comment: epsilon equals 0.001 -- detecte +sieurs points
            double epsilon = 3.0;
            // if the center of the dot is the same than the current
            // don't add it, test the next point of the grid
            cogTmpDot = tmpDot.getCog();

            if ((fabs(cogTmpDot.get_u() - cogDotToTest.get_u()) < epsilon) &&
                (fabs(cogTmpDot.get_v() - cogDotToTest.get_v()) < epsilon)) {
              stopLoop = true;
              // Jump all the pixels between v,u and v,
              // tmpDot->getFirstBorder_u()
              data.m_u = border_u;
              data.m_v = border_v;
            }
            else {
              double otherDiff_u = cogTmpDot.get_u() - area_center_u;
              double otherDiff_v = cogTmpDot.get_v() - area_center_v;
              double otherDist = sqrt((otherDiff_u * otherDiff_u) + (otherDiff_v * otherDiff_v));

              // if the distance of the curent vector element to the center
              // is greater than the distance of this dot to the center,
              // then add this dot before the current vector element.
              if (otherDist > thisDist) {
                data.m_niceDots.insert(itnice, *dotToTest);
                stopLoop = true;
                // Jump all the pixels between v,u and v,
                // tmpDot->getFirstBorder_u()
                data.m_u = border_u;
                data.m_v = border_v;
              }
              ++itnice;
            }
          }

          // if we reached the end of the vector without finding the dot
          // or inserting it, insert it now.
          if ((itnice == data.m_niceDots.end()) && (stopLoop == false)) {
            data.m_niceDots.push_back(*dotToTest);
          }
        }
        else {
          // Store bad dots
          data.m_badDotsVector.push_front(*dotToTest);
        }
      }
    }
  }
  if (dotToTest != nullptr) {
    delete dotToTest;
  }
}

/*!

  Look for a list of dot matching this dot parameters within a region of
  interest defined by a rectangle in the image. The rectangle upper-left
  coordinates are given by
  (\e area_u, \e area_v). The size of the rectangle is given by \e area_w and
  \e area_h.

  \param I : Image to process.
  \param area_u : Coordinate (column) of the upper-left area corner.
  \param area_v : Coordinate (row) of the upper-left area corner.

  \param area_w : Width or the area in which a dot is searched.
  \param area_h : Height or the area in which a dot is searched.

  \param niceDots: List of the dots that are found.

  \warning Allocates memory for the list of vpDot2 returned by this method.
  Desallocation has to be done by yourself, see searchDotsInArea()

  \sa searchDotsInArea(vpImage<unsigned char>& I, std::list<vpDot2> &)
*/
void vpDot2::searchDotsInArea(const vpImage<unsigned char> &I, int area_u, int area_v, unsigned int area_w,
                              unsigned int area_h, std::list<vpDot2> &niceDots)

{
  // clear the list of nice dots
  niceDots.clear();

  // Fit the input area in the image; we keep only the common part between
  // this area and the image.
  setArea(I, area_u, area_v, area_w, area_h);

  // compute the size of the search grid
  unsigned int gridWidth;
  unsigned int gridHeight;
  getGridSize(gridWidth, gridHeight);

  if (m_graphics) {
    // Display the area were the dot is search
    vpDisplay::displayRectangle(I, m_area, vpColor::blue, false, m_thickness);
  }

#ifdef DEBUG
  vpDisplay::displayRectangle(I, m_area, vpColor::blue);
  vpDisplay::flush(I);
#endif
  // start the search loop; for all points of the search grid,
  // test if the pixel belongs to a valid dot.
  // if it is so eventually add it to the vector of valid dots.
  std::list<vpDot2> badDotsVector;
  std::list<vpDot2>::iterator itnice;

  vpDot2 tmpDot;

  unsigned int area_u_min = static_cast<unsigned int>(m_area.getLeft());
  unsigned int area_u_max = static_cast<unsigned int>(m_area.getRight());
  unsigned int area_v_min = static_cast<unsigned int>(m_area.getTop());
  unsigned int area_v_max = static_cast<unsigned int>(m_area.getBottom());

  unsigned int u, v;
  vpImagePoint cogTmpDot;

  v = area_v_min;
  while (v < area_v_max) {
    u = area_u_min;
    while (u < area_u_max) {
      // if the pixel we're in doesn't have the right color (outside the
      // graylevel interval), no need to check further, just get to the
      // next grid intersection.
      if (hasGoodLevel(I, u, v)) {

        // Test if an other germ is inside the bounding box of a dot previously
        // detected
        bool good_germ = true;

        itnice = niceDots.begin();
        while ((itnice != niceDots.end()) && (good_germ == true)) {
          tmpDot = *itnice;

          cogTmpDot = tmpDot.getCog();
          double u0 = cogTmpDot.get_u();
          double v0 = cogTmpDot.get_v();
          double half_w = tmpDot.getWidth() / 2.;
          double half_h = tmpDot.getHeight() / 2.;

          if ((u >= (u0 - half_w)) && (u <= (u0 + half_w)) && (v >= (v0 - half_h)) && (v <= (v0 + half_h))) {
            // Germ is in a previously detected dot
            good_germ = false;
          }
          ++itnice;
        }

        if (good_germ) {
          vpRect area(area_u, area_v, area_w, area_h);
          vpSearchDotsInAreaGoodGermData data(I, area, u, v, niceDots, badDotsVector);
          searchDotsAreaGoodGerm(data);
        }
      }
      u = u + gridWidth;
    }
    v = v + gridHeight;
  }
}

END_VISP_NAMESPACE
