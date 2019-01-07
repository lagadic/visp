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
 * Template tracker.
 *
 * Authors:
 * Amaury Dame
 * Aurelien Yol
 * Fabien Spindler
 *
 *****************************************************************************/
#include <visp3/tt/vpTemplateTrackerTriangle.h>

/*!
  Default constructor.
 */
vpTemplateTrackerTriangle::vpTemplateTrackerTriangle()
  : minx_temp(0), miny_temp(0), C1(), C2(), C3(), l_t(0), h_t(0), not_good(false), uvinv00(0.), uvinv01(0.),
    uvinv10(0.), uvinv11(0.), marge_triangle(0.00001), area(0)
{
}

/*!
  Copy constructor.
 */
vpTemplateTrackerTriangle::vpTemplateTrackerTriangle(const vpTemplateTrackerTriangle &T)
  : minx_temp(0), miny_temp(0), C1(), C2(), C3(), l_t(0), h_t(0), not_good(false), uvinv00(0.), uvinv01(0.),
    uvinv10(0.), uvinv11(0.), marge_triangle(0.00001), area(0)
{
  *this = T;
}

/*!
  Copy operator.
 */
vpTemplateTrackerTriangle &vpTemplateTrackerTriangle::operator=(const vpTemplateTrackerTriangle &T)
{
  minx_temp = T.minx_temp;
  miny_temp = T.miny_temp;

  l_t = T.l_t;
  h_t = T.h_t;
  C1.x = T.C1.x;
  C1.y = T.C1.y;
  C2.x = T.C2.x;
  C2.y = T.C2.y;
  C3.x = T.C3.x;
  C3.y = T.C3.y;
  // uvinv.resize(2,2);
  // uvinv=T.uvinv;
  // p_ds_uv.resize(2);
  // p_ds_uv=T.p_ds_uv;
  // ptempo.resize(2);
  // ptempo=T.ptempo;
  not_good = T.not_good;

  uvinv00 = T.uvinv00;
  uvinv01 = T.uvinv01;
  uvinv10 = T.uvinv10;
  uvinv11 = T.uvinv11;

  marge_triangle = T.marge_triangle;
  area = T.area;

  return (*this);
}

/*!
  Create a triangle from 3 corners.
  \param c1 : First corner.
  \param c2 : Second corner.
  \param c3 : Third corner.

  The coordinates of the points are defined as a 2 dimension vector with
  coordinates (x,y).
  */
vpTemplateTrackerTriangle::vpTemplateTrackerTriangle(const vpColVector &c1, const vpColVector &c2,
                                                     const vpColVector &c3)
  : minx_temp(0), miny_temp(0), C1(), C2(), C3(), l_t(0), h_t(0), not_good(false), uvinv00(0.), uvinv01(0.),
    uvinv10(0.), uvinv11(0.), marge_triangle(0.00001), area(0)
{
  init(c1[0], c1[1], c2[0], c2[1], c3[0], c3[1]);
}
/*!
  Return a triangle with coordinates that are down scaled by a factor 2.
  */
vpTemplateTrackerTriangle vpTemplateTrackerTriangle::getPyramidDown() const
{
  vpTemplateTrackerTriangle Ttemp;
  Ttemp.init(C1.x / 2., C1.y / 2., C2.x / 2., C2.y / 2., C3.x / 2., C3.y / 2.);
  return Ttemp;
}

/*!
  Create a triangle from 3 corners with coordinates (x1,y1), (x2,y2), (x3,y3).
  - x coordinate is along the columns
  - y coordinate is along the rows.
  */
vpTemplateTrackerTriangle::vpTemplateTrackerTriangle(int x1, int y1, int x2, int y2, int x3, int y3)
  : minx_temp(0), miny_temp(0), C1(), C2(), C3(), l_t(0), h_t(0), not_good(false), uvinv00(0.), uvinv01(0.),
    uvinv10(0.), uvinv11(0.), marge_triangle(0.00001), area(0)
{
  init(x1, y1, x2, y2, x3, y3);
}

/*!
  Create a triangle from 3 corners defined as image points.
  \param c1 : First corner.
  \param c2 : Second corner.
  \param c3 : Third corner.
 */
vpTemplateTrackerTriangle::vpTemplateTrackerTriangle(const vpImagePoint &c1, const vpImagePoint &c2,
                                                     const vpImagePoint &c3)
  : minx_temp(0), miny_temp(0), C1(), C2(), C3(), l_t(0), h_t(0), not_good(false), uvinv00(0.), uvinv01(0.),
    uvinv10(0.), uvinv11(0.), marge_triangle(0.00001), area(0)
{
  init(c1.get_u(), c1.get_v(), c2.get_u(), c2.get_v(), c3.get_u(), c3.get_v());
}

/*!
  Create a triangle from 3 corners with coordinates (x1,y1), (x2,y2), (x3,y3).
  - x coordinate is along the columns
  - y coordinate is along the rows.
  */
vpTemplateTrackerTriangle::vpTemplateTrackerTriangle(double x1, double y1, double x2, double y2, double x3, double y3)
  : minx_temp(0), miny_temp(0), C1(), C2(), C3(), l_t(0), h_t(0), not_good(false), uvinv00(0.), uvinv01(0.),
    uvinv10(0.), uvinv11(0.), marge_triangle(0.00001), area(0)
{
  init(x1, y1, x2, y2, x3, y3);
}
/*!
  Initializes a triangle from 3 corners.
  \param c1 : First corner.
  \param c2 : Second corner.
  \param c3 : Third corner.

  The coordinates of the points are defined as a 2 dimension vector with
  coordinates (x,y).
  */
void vpTemplateTrackerTriangle::init(const vpColVector &c1, const vpColVector &c2, const vpColVector &c3)
{
  init(c1[0], c1[1], c2[0], c2[1], c3[0], c3[1]);
}
/*!
  Initializes a triangle from 3 corners defined as image points.
  \param c1 : First corner.
  \param c2 : Second corner.
  \param c3 : Third corner.
 */
void vpTemplateTrackerTriangle::init(const vpImagePoint &c1, const vpImagePoint &c2, const vpImagePoint &c3)
{
  init(c1.get_u(), c1.get_v(), c2.get_u(), c2.get_v(), c3.get_u(), c3.get_v());
}

/*!
  Initializes a triangle from 3 corners with coordinates (x1,y1), (x2,y2),
  (x3,y3).
  - x coordinate is along the columns
  - y coordinate is along the rows.
  */
void vpTemplateTrackerTriangle::init(int x1, int y1, int x2, int y2, int x3, int y3)
{
  init((double)x1, (double)y1, (double)x2, (double)y2, (double)x3, (double)y3);
}

/*!
  Initializes a triangle from 3 corners with coordinates (x1,y1), (x2,y2),
  (x3,y3).
  - x coordinate is along the columns
  - y coordinate is along the rows.
  */
void vpTemplateTrackerTriangle::init(double x1, double y1, double x2, double y2, double x3, double y3)
{
  C1.x = x1;
  C1.y = y1;
  C2.x = x2;
  C2.y = y2;
  C3.x = x3;
  C3.y = y3;

  double minx, miny, maxx, maxy;
  // calcul du rectangle minimal contenant le triangle seletionne
  minx = (x1 < x2) ? x1 : x2;
  miny = (y1 < y2) ? y1 : y2;
  minx = (minx < x3) ? minx : x3;
  miny = (miny < y3) ? miny : y3;
  maxx = (x1 > x2) ? x1 : x2;
  maxy = (y1 > y2) ? y1 : y2;
  maxx = (maxx > x3) ? maxx : x3;
  maxy = (maxy > y3) ? maxy : y3;

  vpColVector u;
  vpColVector v;
  u.resize(2);
  v.resize(2);
  vpMatrix uv(2, 2);
  vpMatrix uvinv(2, 2);

  u[0] = C2.x - C1.x;
  u[1] = C2.y - C1.y;

  v[0] = C3.x - C1.x;
  v[1] = C3.y - C1.y;

  uv[0][0] = u[0];
  uv[1][0] = v[0];
  uv[0][1] = u[1];
  uv[1][1] = v[1];
  try {
    uvinv = uv.inverseByLU();
    not_good = false;
  } catch (...) {
    not_good = true;
    std::cout << "Triangle vide" << std::endl;
  }
  uvinv00 = uvinv[0][0];
  uvinv01 = uvinv[0][1];
  uvinv10 = uvinv[1][0];
  uvinv11 = uvinv[1][1];

  l_t = maxx - minx;
  h_t = maxy - miny;
  minx_temp = minx;
  miny_temp = miny;

  marge_triangle = 0.00001;
  area = 0.5 * fabs(uv.det());
}

// marge ajoutee a zone pour que sommet soit pris en compte

/*!
  Indicates if a point with coordinates (i,j) is in the triangle.
  \param i : Coordinate along the rows.
  \param j : Coordinate along the columns.
 */
bool vpTemplateTrackerTriangle::inTriangle(const int &i, const int &j) const
{
  if (not_good)
    return false;

  /*ptempo[0]=j-C1.x;
  ptempo[1]=i-C1.y;

  p_ds_uv=ptempo*uvinv;
  return (p_ds_uv[0]+p_ds_uv[1]<1. && p_ds_uv[0]>0 && p_ds_uv[1]>0);*/

  double ptempo0 = j - C1.x;
  double ptempo1 = i - C1.y;
  double p_ds_uv0 = ptempo0 * uvinv00 + ptempo1 * uvinv10;
  double p_ds_uv1 = ptempo0 * uvinv01 + ptempo1 * uvinv11;
  return (p_ds_uv0 + p_ds_uv1 < 1. + marge_triangle && p_ds_uv0 > -marge_triangle && p_ds_uv1 > -marge_triangle);
}

/*!
  Indicates if a point with coordinates (i,j) is in the triangle.
  \param i : Coordinate along the rows.
  \param j : Coordinate along the columns.
 */
bool vpTemplateTrackerTriangle::inTriangle(const double &i, const double &j) const
{
  if (not_good)
    return false;
  /*ptempo[0]=j-C1.x;
  ptempo[1]=i-C1.y;

  p_ds_uv=ptempo*uvinv;
  return (p_ds_uv[0]+p_ds_uv[1]<1. && p_ds_uv[0]>0 && p_ds_uv[1]>0);*/
  double ptempo0 = j - C1.x;
  double ptempo1 = i - C1.y;
  double p_ds_uv0 = ptempo0 * uvinv00 + ptempo1 * uvinv10;
  double p_ds_uv1 = ptempo0 * uvinv01 + ptempo1 * uvinv11;
  return (p_ds_uv0 + p_ds_uv1 < 1. + marge_triangle && p_ds_uv0 > -marge_triangle && p_ds_uv1 > -marge_triangle);
}

/*!
  Indicates if an image point is in the triangle.
  \param ip : Image point to consider.
 */
bool vpTemplateTrackerTriangle::inTriangle(const vpImagePoint &ip) const { return inTriangle(ip.get_i(), ip.get_j()); }
/*!
  Returns the coordinates of the triangle corners as an image point.
  \param c1 : First corner.
  \param c2 : Second corner.
  \param c3 : Third corner.
 */
void vpTemplateTrackerTriangle::getCorners(vpImagePoint &c1, vpImagePoint &c2, vpImagePoint &c3) const
{
  c1.set_uv(C1.x, C1.y);
  c2.set_uv(C2.x, C2.y);
  c3.set_uv(C3.x, C3.y);
}

/*!
  Returns the coordinates of the triangle corners as a 3 dimension vector of
  image points. \param c : 3 dimension vector of image points that correspond
  to the triangle corners.
 */
void vpTemplateTrackerTriangle::getCorners(std::vector<vpImagePoint> &c) const
{
  c.resize(3);
  c[0].set_uv(C1.x, C1.y);
  c[1].set_uv(C2.x, C2.y);
  c[2].set_uv(C3.x, C3.y);
}

/*!
  Returns the coordinates of the triangle corners as a 2 dimension vector
  (x,y). \param c1 : First corner. \param c2 : Second corner. \param c3 :
  Third corner.
 */
void vpTemplateTrackerTriangle::getCorners(vpColVector &c1, vpColVector &c2, vpColVector &c3) const
{
  c1 = getCorner1();
  c2 = getCorner2();
  c3 = getCorner3();
}

/*!
  Returns the coordinates of the triangle first corner.
  \return A vector with dimension 2, that contains the coordinates (x,y) of
  the corner.
 */
vpColVector vpTemplateTrackerTriangle::getCorner1() const
{
  vpColVector c(2);
  c[0] = C1.x;
  c[1] = C1.y;

  return c;
}
/*!
  Returns the coordinates of the triangle second corner.
  \return A vector with dimension 2, that contains the coordinates (x,y) of
  the corner.
 */
vpColVector vpTemplateTrackerTriangle::getCorner2() const
{
  vpColVector c(2);
  c[0] = C2.x;
  c[1] = C2.y;
  return c;
}

/*!
  Returns the coordinates of the triangle third corner.
  \return A vector with dimension 2, that contains the coordinates (x,y) of
  the corner.
 */
vpColVector vpTemplateTrackerTriangle::getCorner3() const
{
  vpColVector c(2);
  c[0] = C3.x;
  c[1] = C3.y;
  return c;
}

/*!
  Get the size of the triangle bounding box.
  \param w : Bounding box width.
  \param h : Bounding box height.
 */
void vpTemplateTrackerTriangle::getSize(double &w, double &h) const
{
  w = l_t;
  h = h_t;
}
/*!
  Get the size of the triangle bounding box.
  \param w : Bounding box width.
  \param h : Bounding box height.
 */
void vpTemplateTrackerTriangle::getSize(int &w, int &h) const
{
  w = (int)l_t + 1;
  h = (int)h_t + 1;
}

/*!
  \return The minimal x coordinate (along the columns of the image) of the
  points that are in the triangle. \sa getMaxx()
 */
double vpTemplateTrackerTriangle::getMinx() const { return minx_temp - 1; }
/*!
  \return The minimal y coordinate (along the rows of the image) of the points
  that are in the triangle. \sa getMaxy()
 */
double vpTemplateTrackerTriangle::getMiny() const { return miny_temp - 1; }
/*!
  \return The maximal x coordinate (along the columns of the image) of the
  points that are in the triangle. \sa getMinx()
 */
double vpTemplateTrackerTriangle::getMaxx() const { return minx_temp + l_t + 1; }
/*!
  \return The maximal y coordinate (along the rows of the image) of the points
  that are in the triangle. \sa getMaxx()
 */
double vpTemplateTrackerTriangle::getMaxy() const { return miny_temp + h_t + 1; }
