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
 * 2D point useful for image processing
 */

/*!
  \file vpImagePoint.h
  \brief Class that defines a 2D point in an image. This class is useful
  for image processing
 */

#ifndef VP_IMAGE_POINT_H
#define VP_IMAGE_POINT_H

#include <visp3/core/vpConfig.h>

#include <cmath>  // std::fabs
#include <ostream>
#include <vector>

BEGIN_VISP_NAMESPACE
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
  inline vpImagePoint() : i(0), j(0) { }
  /*!
    Default constructor that initialize the coordinates of the image
    thanks to the parameters \f$ ii \f$ and \f$ jj \f$.
  */
  inline vpImagePoint(double ii, double jj) : i(ii), j(jj) { }
  /*!
    Copy constructor.

    Initialize the coordinates of the image point with \e ip.

    \param ip : An image point.
  */
  inline vpImagePoint(const vpImagePoint &ip) : i(ip.i), j(ip.j) { }
  //! Destructor.
  inline virtual ~vpImagePoint() { }

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

  bool inRectangle(const vpRect &rect) const;

  /*!
   * Test if the image point belongs to a segment represented by two image points.
   *
   * \param[in] start : Segment start image point.
   * \param[in] end : Segment end image point.
   * \return True if current image point belongs to the segment. False otherwise.
   *
   * To see how to use this function, a code snippet is given in nextInSegment().
   *
   * \sa nextInSegment()
   */
  inline bool inSegment(const vpImagePoint &start, const vpImagePoint &end) const
  {
    bool cond11 = ((end.get_j() >= start.get_j()) && (end.get_j() >= this->j) && (this->j >= start.get_j()));
    bool cond12 = ((end.get_j() <= start.get_j()) && (end.get_j() <= this->j) && (this->j <= start.get_j()));
    bool cond21 = ((end.get_i() >= start.get_i()) && (end.get_i() >= this->i) && (this->i >= start.get_i()));
    bool cond22 = ((end.get_i() <= start.get_i()) && (end.get_i() <= this->i) && (this->i <= start.get_i()));
    return (cond11 || cond12) && (cond21 || cond22);
  }

  /*!
   * Considering current image point, returns the next image point that belongs to the segment [start,end].
   *
   * \param[in] start : Segment start image point.
   * \param[in] end : Segment end image point.
   * \return Regarding current image point, next image point that belongs to the liSegmentne [start,end].
   *
   * The following sample code shows how to use this function to find all the pixels that belong
   * to the segment defined by 2 image points with coordinates [10,12] and [20,16]:
   * \code
   * #include <iostream>
   * #include <visp3/core/vpImagePoint.h>
   *
   * #ifdef ENABLE_VISP_NAMESPACE
   * using namespace VISP_NAMESPACE_NAME;
   * #endif
   *
   * int main()
   * {
   *   vpImagePoint start_pixel(10, 12);
   *   vpImagePoint end_pixel(20, 16);
   *
   *   for (auto curr_pixel = start_pixel; curr_pixel.inSegment(start_pixel, end_pixel);
   *     curr_pixel = curr_pixel.nextInSegment(start_pixel, end_pixel)) {
   *     std::cout << "pixel: " << curr_pixel << std::endl;
   *     if (curr_pixel == end_pixel) break;
   *   }
   *
   *   return EXIT_SUCCESS;
   * }
   * \endcode
   *
   * It produces the following output by printing all the pixels belonging to the segment:
   * \code
   * pixel: 10, 12
   * pixel: 11, 12.4
   * pixel: 12, 12.8
   * pixel: 13, 13.2
   * pixel: 14, 13.6
   * pixel: 15, 14
   * pixel: 16, 14.4
   * pixel: 17, 14.8
   * pixel: 18, 15.2
   * pixel: 19, 15.6
   * pixel: 20, 16
   * \endcode
   *
   * \sa inSegment()
   */
  inline vpImagePoint nextInSegment(const vpImagePoint &start, const vpImagePoint &end) const
  {
    const double line_slope = (end.get_i() - start.get_i()) / (end.get_j() - start.get_j());
    if (fabs(end.get_j() - this->j) > fabs(end.get_i() - this->i)) {
      double j_ = (end.get_j() > this->j ? (this->j + 1) : (this->j - 1));
#if ((__cplusplus >= 201103L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201103L))) // Check if cxx11 or higher
      return { end.get_i() - (line_slope * (end.get_j() - j_)), j_ };
#else
      return vpImagePoint(end.get_i() - (line_slope * (end.get_j() - j_)), j_);
#endif
    }
    else {
      double i_ = (end.get_i() > this->i ? (this->i + 1) : (this->i - 1));
#if ((__cplusplus >= 201103L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201103L))) // Check if cxx11 or higher
      return { i_, end.get_j() - ((end.get_i() - i_) / line_slope) };
#else
      return vpImagePoint(i_, end.get_j() - ((end.get_i() - i_) / line_slope));
#endif
    }
  }

  /*!
    Copy operator.
  */
  inline vpImagePoint &operator=(const vpImagePoint &ip)
  {
    this->i = ip.i;
    this->j = ip.j;
    return *this;
  }

#if ((__cplusplus >= 201103L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201103L))) // Check if cxx11 or higher
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

  static double distance(const vpImagePoint &iP1, const vpImagePoint &iP2);
  static vpRect getBBox(const std::vector<vpImagePoint> &ipVec);
  static double sqrDistance(const vpImagePoint &iP1, const vpImagePoint &iP2);

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
END_VISP_NAMESPACE
#endif
