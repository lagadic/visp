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
 * Template tracker.
 *
 * Authors:
 * Amaury Dame
 * Aurelien Yol
 * Fabien Spindler
 *
 *****************************************************************************/
/*!
 \file vpTemplateTrackerTriangle.h
 \brief
*/

#ifndef vpTemplateTrackerTriangle_hh
#define vpTemplateTrackerTriangle_hh

#include <assert.h>
#include <vector>

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/tt/vpTemplateTrackerHeader.h>

/*!
  \class vpTemplateTrackerTriangle
  \ingroup group_tt_tools
*/
class VISP_EXPORT vpTemplateTrackerTriangle
{
protected:
  double minx_temp;
  double miny_temp;
  vpTemplateTrackerDPoint C1; //! Corner 1
  vpTemplateTrackerDPoint C2; //! Corner 2
  vpTemplateTrackerDPoint C3; //! Corner 3

  double l_t;
  double h_t;

  bool not_good;
  double uvinv00;
  double uvinv01;
  double uvinv10;
  double uvinv11;
  double marge_triangle;
  double area;

private:
  vpColVector getCorner1() const;
  vpColVector getCorner2() const;
  vpColVector getCorner3() const;

public:
  vpTemplateTrackerTriangle();
  vpTemplateTrackerTriangle(const vpTemplateTrackerTriangle &T);
  vpTemplateTrackerTriangle(const vpColVector &c1, const vpColVector &c2, const vpColVector &c3);
  vpTemplateTrackerTriangle(const vpImagePoint &c1, const vpImagePoint &c2, const vpImagePoint &c3);
  vpTemplateTrackerTriangle(int x1, int y1, int x2, int y2, int x3, int y3);
  vpTemplateTrackerTriangle(double x1, double y1, double x2, double y2, double x3, double y3);

  /*!
    Return the area of the triangle.

    \return The area of the triangle.
  */
  inline double getArea() const { return this->area; }

  vpTemplateTrackerTriangle getPyramidDown() const;
  void getCorners(vpColVector &c1, vpColVector &c2, vpColVector &c3) const;
  void getCorners(vpImagePoint &c1, vpImagePoint &c2, vpImagePoint &c3) const;
  void getCorners(std::vector<vpImagePoint> &c) const;

  /*!
     Returns the coordinates of a triangle corner.
     \param i : Allowed values are 0, 1 or 2.
     \return
     - if i = 0, return corner 1 coordinates,
     - if i = 1, return corner 2 coordinates,
     - if i = 2, return corner 3 coordinates.

     The coordinates are returned as a 2 dimension vector (x,y).
   */
  vpColVector getCorner(unsigned int i) const
  {
    assert(i < 3);
    if (i == 0)
      return getCorner1();
    else if (i == 1)
      return getCorner2();
    else /*if(i==2)*/
      return getCorner3();
  };
  /*!
     Returns the coordinates of a triangle corner.
     \param i : Allowed values are 0, 1 or 2.
     - if i = 0, return corner 1 coordinates,
     - if i = 1, return corner 2 coordinates,
     - if i = 2, return corner 3 coordinates.
     \param x : The coordinate of the corner along horizontal axis.
     \param y : The coordinate of the corner along vertical axis.
   */
  void getCorner(unsigned int i, double &x, double &y) const
  {
    assert(i < 3);
    if (i == 0) {
      x = C1.x;
      y = C1.y;
    } else if (i == 1) {
      x = C2.x;
      y = C2.y;
    } else /*if(i==2)*/ {
      x = C3.x;
      y = C3.y;
    }
  };

  double getMaxx() const;
  double getMaxy() const;
  double getMinx() const;
  double getMiny() const;

  void getSize(double &w, double &h) const;
  void getSize(int &w, int &h) const;

  void init(const vpColVector &c1, const vpColVector &c2, const vpColVector &c3);
  void init(const vpImagePoint &c1, const vpImagePoint &c2, const vpImagePoint &c3);
  void init(int x1, int y1, int x2, int y2, int x3, int y3);
  void init(double x1, double y1, double x2, double y2, double x3, double y3);
  bool inTriangle(const vpImagePoint &ip) const;
  bool inTriangle(const int &i, const int &j) const;
  bool inTriangle(const double &i, const double &j) const;

  vpTemplateTrackerTriangle &operator=(const vpTemplateTrackerTriangle &T);
};
#endif
