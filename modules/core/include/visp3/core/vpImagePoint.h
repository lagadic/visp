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
 * 2D point useful for image processing
 *
 * Authors:
 * Nicolas Melchior
 * Fabien Spindler
 *
 *****************************************************************************/

#ifndef vpImagePoint_H
#define vpImagePoint_H

/*!
  \file vpImagePoint.h
  \brief Class that defines a 2D point in an image. This class is useful
  for image processing
*/

#include <visp3/core/vpConfig.h>

#include <cmath>  // std::fabs
#include <limits> // numeric_limits
#include <ostream>
#include <vector>

class vpRect;

/*!
  \class vpImagePoint
  \ingroup group_core_image

  \brief Class that defines a 2D point in an image. This class is
  useful for image processing and stores only the <B>2D coordinates
  given in sub-pixel</B>.

  \warning If you want to define a point thanks to its coordinates
  given in meter in the object frame, the camera frame or the image
  plane, you have to use the class vpPoint.

  In this class, the 2D coordinates are not necessary integer
  values. It is easy to manipulate the given coordinates in the two
  frames used in ViSP : the (i,j) coordinates and the (u,v)
  coordinates.  The two following images illustrate the two coordinate
  systems.

  \image html vpImagePoint.gif
  \image latex vpImagePoint.ps  width=10cm

  \warning <B>An instance of the vpImagePoint class corresponds to a
  particular point. Thus, if you change the point coordinate using the
  method set_i(double i), it produces the same effect than if
  you used the method set_v(double v). These two methods change
  the same private attribute. It is also true for the two methods
  set_j(double j) and set_u(double u).</B>
*/

class VISP_EXPORT vpImagePoint
{
public:
  /*!
    Default constructor that initialize the coordinates of the image
    point to zero.
  */
  inline vpImagePoint() : i(0), j(0) {}
  /*!
    Default constructor that initialize the coordinates of the image
    thanks to the parameters \f$ ii \f$ and \f$ jj \f$.
  */
  inline vpImagePoint(double ii, double jj) : i(ii), j(jj) {}
  /*!
    Copy constructor.

    Initialize the coordinates of the image point with \e ip.

    \param ip : An image point.
  */
  inline vpImagePoint(const vpImagePoint &ip) : i(ip.i), j(ip.j) {}
  //! Destructor.
  inline virtual ~vpImagePoint() {}

  /*!
    Copy operator.
  */
  inline vpImagePoint &operator=(const vpImagePoint &ip)
  {
    this->i = ip.i;
    this->j = ip.j;
    return *this;
  }
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  /*!
    Move operator.
  */
  inline vpImagePoint &operator=(const vpImagePoint &&ip) noexcept
  {
    this->i = ip.i;
    this->j = ip.j;
    return *this;
  }
#endif

  vpImagePoint &operator+=(const vpImagePoint &ip);

  /*!

    Operator -=.

  */
  inline vpImagePoint &operator-=(const vpImagePoint &ip)
  {
    this->i -= ip.i;
    this->j -= ip.j;
    return *this;
  }
  vpImagePoint &operator/=(double scale);
  /*!

    Operator *=.
  */
  inline vpImagePoint &operator*=(double scale)
  {
    this->i *= scale;
    this->j *= scale;
    return *this;
  }

  /*!

    Sets the point coordinate corresponding to the \f$ i \f$ axes in
    the frame (i,j).

    \param ii : The desired value for the coordinate along the \f$ i \f$ axes.

    \sa set_j(), set_u(), set_v()
  */
  inline void set_i(double ii) { this->i = ii; }

  /*!

    Sets the point coordinate corresponding to the \f$ j \f$ axes in
    the frame (i,j).

    \param jj : The desired value for the coordinate along the \f$ j \f$ axes.

    \sa set_i(), set_u(), set_v()
  */
  inline void set_j(double jj) { this->j = jj; }

  /*!

    Sets the point coordinates in the frame (i,j).

    \param ii : The desired value for the coordinate along the \f$ i \f$ axes.
    \param jj : The desired value for the coordinate along the \f$ j \f$ axes.

    \sa set_i(), set_j(), set_u(), set_v()
  */
  inline void set_ij(double ii, double jj)
  {
    this->i = ii;
    this->j = jj;
  }

  /*!

    Gets the point coordinate corresponding to the \f$ i \f$ axes in
    the frame (i,j).

    \return The value of the coordinate along the \f$ i \f$ axes.

    \sa get_j(), get_u(), get_v()
  */
  inline double get_i() const { return i; }

  /*!

    Gets the point coordinate corresponding to the \f$ j \f$ axes in
    the frame (i,j).

    \return The value of the coordinate along the \f$ j \f$ axes.

    \sa get_i(), get_u(), get_v()
  */
  inline double get_j() const { return j; }

  /*!

    Sets the point coordinate corresponding to the \f$ u \f$ axes in
    the frame (u,v).

    \param u : The desired value for the coordinate along the \f$ u \f$ axes.

    \sa set_i(), set_j(), set_v()
  */
  inline void set_u(double u) { j = u; }

  /*!

    Sets the point coordinate corresponding to the \f$ v \f$ axes in
    the frame (u,v).

    \param v : The desired value for the coordinate along the \f$ v \f$ axes.

    \sa set_i(), set_j(), set_u()
  */
  inline void set_v(double v) { i = v; }

  /*!

    Sets the point coordinates in the frame (u,v).

    \param u : The desired value for the coordinate along the \f$ u \f$ axes.
    \param v : The desired value for the coordinate along the \f$ v \f$ axes.

    \sa set_i(), set_j(), set_u(), set_v()
  */
  inline void set_uv(double u, double v)
  {
    this->i = v;
    this->j = u;
  }

  /*!

    Gets the point coordinate corresponding to the \f$ u \f$ axes in
    the frame (u,v).

    \return The value of the coordinate along the \f$ u \f$ axes.

    \sa get_i(), get_j(), get_v()
  */
  inline double get_u() const { return j; }

  /*!

    Gets the point coordinate corresponding to the \f$ v \f$ axes in
    the frame (u,v).

    \return The value of the coordinate along the \f$ v \f$ axes.

    \sa get_i(), get_j(), get_u()
  */
  inline double get_v() const { return i; }

  /*!
   * Test if the image point is in a line represented by two image points.
   *
   * \param[in] start : Line start image point.
   * \param[in] end : Line end image point.
   * \return True if current image point is in the line. False otherwise.
   */
  inline bool isInLine(const vpImagePoint &start, const vpImagePoint &end) const
  {
    return ((end.get_j() >= start.get_j() && end.get_j() >= this->j && this->j >= start.get_j()) ||
            (end.get_j() <= start.get_j() && end.get_j() <= this->j && this->j <= start.get_j())) &&
           ((end.get_i() >= start.get_i() && end.get_i() >= this->i && this->i >= start.get_i()) ||
            (end.get_i() <= start.get_i() && end.get_i() <= this->i && this->i <= start.get_i()));
  }

  /*!
   * Return next image point of the current image point in the line [start,end].
   *
   * \param[in] start : Line start image point.
   * \param[in] end : Line end image point.
   * \return Next image point regarding current image point in the line [start,end].
   */
  inline vpImagePoint nextInLine(const vpImagePoint &start, const vpImagePoint &end) const
  {
    const double line_slope = (end.get_i() - start.get_i()) / (end.get_j() - start.get_j());
    if (fabs(end.get_j() - this->j) > fabs(end.get_i() - this->i)) {
#if (VISP_CXX_STANDARD > VISP_CXX_STANDARD_98)
      return {end.get_i() - line_slope * (end.get_j() - this->j), end.get_j() > this->j ? this->j + 1 : this->j - 1};
#else
      return vpImagePoint(end.get_i() - line_slope * (end.get_j() - this->j),
                          end.get_j() > this->j ? this->j + 1 : this->j - 1);
#endif
    } else {
#if (VISP_CXX_STANDARD > VISP_CXX_STANDARD_98)
      return {end.get_i() > this->i ? this->i + 1 : this->i - 1, end.get_j() - ((end.get_i() + this->i) / line_slope)};
#else
      return vpImagePoint(end.get_i() > this->i ? this->i + 1 : this->i - 1,
                          end.get_j() - ((end.get_i() + this->i) / line_slope));
#endif
    }
  }

  static vpRect getBBox(const std::vector<vpImagePoint> &ipVec);

  static double distance(const vpImagePoint &iP1, const vpImagePoint &iP2);
  static double sqrDistance(const vpImagePoint &iP1, const vpImagePoint &iP2);

  bool inRectangle(const vpRect &rect) const;

  friend VISP_EXPORT bool operator==(const vpImagePoint &ip1, const vpImagePoint &ip2);
  friend VISP_EXPORT bool operator!=(const vpImagePoint &ip1, const vpImagePoint &ip2);
  friend VISP_EXPORT vpImagePoint operator+=(const vpImagePoint &ip1, const vpImagePoint &ip2);
  friend VISP_EXPORT vpImagePoint operator+(const vpImagePoint &ip1, const vpImagePoint &ip2);
  friend VISP_EXPORT vpImagePoint operator+(const vpImagePoint &ip1, int offset);
  friend VISP_EXPORT vpImagePoint operator+(const vpImagePoint &ip1, unsigned int offset);
  friend VISP_EXPORT vpImagePoint operator+(const vpImagePoint &ip1, double offset);
  friend VISP_EXPORT vpImagePoint operator-(const vpImagePoint &ip1, const vpImagePoint &ip2);
  friend VISP_EXPORT vpImagePoint operator-(const vpImagePoint &ip1, int offset);
  friend VISP_EXPORT vpImagePoint operator-(const vpImagePoint &ip1, unsigned int offset);
  friend VISP_EXPORT vpImagePoint operator-(const vpImagePoint &ip1, double offset);
  friend VISP_EXPORT vpImagePoint operator*(const vpImagePoint &ip1, double scale);
  friend VISP_EXPORT vpImagePoint operator/(const vpImagePoint &ip1, double scale);
  friend VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpImagePoint &ip);

private:
  double i, j;
};

#endif
