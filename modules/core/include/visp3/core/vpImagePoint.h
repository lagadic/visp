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
#include <visp3/core/vpMath.h>

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
  method set_i(const double i), it produces the same effect than if
  you used the method set_v(const double v). These two methods change
  the same private attribute. It is also true for the two methods
  set_j(const double j) and set_u(const double u).</B>
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
  inline virtual ~vpImagePoint() { ; }

  /*!
    Copy operator.
  */
  inline vpImagePoint &operator=(const vpImagePoint &ip)
  {
    this->i = ip.i;
    this->j = ip.j;
    return *this;
  }
#ifdef VISP_HAVE_CPP11_COMPATIBILITY
  /*!
    Move operator.
  */
  inline vpImagePoint &operator=(const vpImagePoint &&ip)
  {
    this->i = std::move(ip.i);
    this->j = std::move(ip.j);
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
  vpImagePoint &operator/=(const double scale);
  /*!

    Operator *=.
*/
  inline vpImagePoint &operator*=(const double scale)
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
  inline void set_i(const double ii) { this->i = ii; }

  /*!

    Sets the point coordinate corresponding to the \f$ j \f$ axes in
    the frame (i,j).

    \param jj : The desired value for the coordinate along the \f$ j \f$ axes.

    \sa set_i(), set_u(), set_v()
  */
  inline void set_j(const double jj) { this->j = jj; }

  /*!

    Sets the point coordinates in the frame (i,j).

    \param ii : The desired value for the coordinate along the \f$ i \f$ axes.
    \param jj : The desired value for the coordinate along the \f$ j \f$ axes.

    \sa set_i(), set_j(), set_u(), set_v()
  */
  inline void set_ij(const double ii, const double jj)
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
  inline void set_u(const double u) { j = u; }

  /*!

    Sets the point coordinate corresponding to the \f$ v \f$ axes in
    the frame (u,v).

    \param v : The desired value for the coordinate along the \f$ v \f$ axes.

    \sa set_i(), set_j(), set_u()
  */
  inline void set_v(const double v) { i = v; }

  /*!

    Sets the point coordinates in the frame (u,v).

    \param u : The desired value for the coordinate along the \f$ u \f$ axes.
    \param v : The desired value for the coordinate along the \f$ v \f$ axes.

    \sa set_i(), set_j(), set_u(), set_v()
  */
  inline void set_uv(const double u, const double v)
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

    Compute the distance \f$ |iP1 - iP2| = \sqrt{(i_1-i_2)^2+(j_1-j_2)^2} \f$

    \param iP1 : First point
    \param iP2 : Second point

    \return the distance between the two points.
  */
  static double distance(const vpImagePoint &iP1, const vpImagePoint &iP2)
  {
    return (sqrt(vpMath::sqr(iP1.get_i() - iP2.get_i()) + vpMath::sqr(iP1.get_j() - iP2.get_j())));
  }

  static vpRect getBBox(const std::vector<vpImagePoint> &ipVec);

  /*!

    Compute the distance \f$ |iP1 - iP2| = (i_1-i_2)^2+(j_1-j_2)^2 \f$

    \param iP1 : First point
    \param iP2 : Second point

    \return the distance between the two points.
  */
  static double sqrDistance(const vpImagePoint &iP1, const vpImagePoint &iP2)
  {
    return (vpMath::sqr(iP1.get_i() - iP2.get_i()) + vpMath::sqr(iP1.get_j() - iP2.get_j()));
  }

  bool inRectangle(const vpRect &rect) const;

  friend VISP_EXPORT bool operator==(const vpImagePoint &ip1, const vpImagePoint &ip2);
  friend VISP_EXPORT bool operator!=(const vpImagePoint &ip1, const vpImagePoint &ip2);
  friend VISP_EXPORT vpImagePoint operator+=(const vpImagePoint &ip1, const vpImagePoint &ip2);
  friend VISP_EXPORT vpImagePoint operator+(const vpImagePoint &ip1, const vpImagePoint &ip2);
  friend VISP_EXPORT vpImagePoint operator+(const vpImagePoint &ip1, const int offset);
  friend VISP_EXPORT vpImagePoint operator+(const vpImagePoint &ip1, const unsigned int offset);
  friend VISP_EXPORT vpImagePoint operator+(const vpImagePoint &ip1, const double offset);
  friend VISP_EXPORT vpImagePoint operator-(const vpImagePoint &ip1, const vpImagePoint &ip2);
  friend VISP_EXPORT vpImagePoint operator-(const vpImagePoint &ip1, const int offset);
  friend VISP_EXPORT vpImagePoint operator-(const vpImagePoint &ip1, const unsigned int offset);
  friend VISP_EXPORT vpImagePoint operator-(const vpImagePoint &ip1, const double offset);
  friend VISP_EXPORT vpImagePoint operator*(const vpImagePoint &ip1, const double scale);
  friend VISP_EXPORT vpImagePoint operator/(const vpImagePoint &ip1, const double scale);
  friend VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpImagePoint &ip);

private:
  double i, j;
};

#endif
