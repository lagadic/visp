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
 * Track a white dot.
 */

/*!
  \file vpDot2.cpp
  \brief Track a dot.
*/

#include <visp3/core/vpDisplay.h>

// exception handling
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpTrackingException.h>

#include <cmath> // std::fabs
#include <iostream>
#include <limits> // numeric_limits
#include <math.h>
#include <visp3/blob/vpDot2.h>

BEGIN_VISP_NAMESPACE

/******************************************************************************
 *
 *      CONSTRUCTORS AND DESTRUCTORS
 *
*****************************************************************************/
/*!

  Initialize the tracker with default parameters.

*/
void vpDot2::init()
{
  const unsigned int val_max = 255;
  const unsigned int val_median = 128;
  m_cog.set_u(0);
  m_cog.set_v(0);

  m_width = 0;
  m_height = 0;
  m_surface = 0;
  m_mean_gray_level = 0;
  m_gray_level_min = val_median;
  m_gray_level_max = val_max;
  m_grayLevelPrecision = 0.80;
  m_gamma = 1.5;

  m_sizePrecision = 0.65;
  m_ellipsoidShapePrecision = 0.65;
  m_maxSizeSearchDistPrecision = 0.65;
  setEllipsoidBadPointsPercentage();
  m00 = 0.;
  m11 = 0.;
  m02 = 0.;
  m20 = 0.;
  m10 = 0.;
  m01 = 0.;
  mu11 = 0.;
  mu02 = 0.;
  mu20 = 0.;

  m_bbox_u_min = 0;
  m_bbox_u_max = 0;
  m_bbox_v_min = 0;
  m_bbox_v_max = 0;

  m_firstBorder_u = 0;
  m_firstBorder_v = 0;

  m_compute_moment = false;
  m_graphics = false;
  m_thickness = 1;
}

/*!
  Default constructor. Just do basic default initialization.
*/
vpDot2::vpDot2()
  : m00(0.), m10(0.), m01(0.), m11(0.), m20(0.), m02(0.), mu11(0.), mu20(0.), mu02(0.), m_cog(), m_width(0), m_height(0),
  m_surface(0), m_mean_gray_level(0), m_grayLevelPrecision(0.8), m_gamma(1.5),
  m_sizePrecision(0.65), m_ellipsoidShapePrecision(0.65), m_maxSizeSearchDistPrecision(0.65),
  m_allowedBadPointsPercentage(0.), m_area(), m_direction_list(), m_ip_edges_list(), m_compute_moment(false), m_graphics(false),
  m_thickness(1), m_bbox_u_min(0), m_bbox_u_max(0), m_bbox_v_min(0), m_bbox_v_max(0), m_firstBorder_u(0), m_firstBorder_v()
{
  const unsigned int val_max = 255;
  const unsigned int val_median = 128;
  m_gray_level_min = val_median;
  m_gray_level_max = val_max;
}

/*!

  Constructor initialize the coordinates of the gravity center of the dot to
  the image point \e ip.  Rest is the same as the default constructor.

  \param ip : An image point with sub-pixel coordinates.

*/
vpDot2::vpDot2(const vpImagePoint &ip)
  : m00(0.), m10(0.), m01(0.), m11(0.), m20(0.), m02(0.), mu11(0.), mu20(0.), mu02(0.), m_cog(ip), m_width(0), m_height(0),
  m_surface(0), m_mean_gray_level(0), m_grayLevelPrecision(0.8), m_gamma(1.5),
  m_sizePrecision(0.65), m_ellipsoidShapePrecision(0.65), m_maxSizeSearchDistPrecision(0.65),
  m_allowedBadPointsPercentage(0.), m_area(), m_direction_list(), m_ip_edges_list(), m_compute_moment(false), m_graphics(false),
  m_thickness(1), m_bbox_u_min(0), m_bbox_u_max(0), m_bbox_v_min(0), m_bbox_v_max(0), m_firstBorder_u(0), m_firstBorder_v()
{
  const unsigned int val_max = 255;
  const unsigned int val_median = 128;
  m_gray_level_min = val_median;
  m_gray_level_max = val_max;
}

/*!
  Copy constructor.
*/
vpDot2::vpDot2(const vpDot2 &twinDot)
  : vpTracker(twinDot), m00(0.), m10(0.), m01(0.), m11(0.), m20(0.), m02(0.), mu11(0.), mu20(0.), mu02(0.), m_cog(),
  m_width(0), m_height(0), m_surface(0), m_mean_gray_level(0),
  m_grayLevelPrecision(0.8), m_gamma(1.5), m_sizePrecision(0.65), m_ellipsoidShapePrecision(0.65),
  m_maxSizeSearchDistPrecision(0.65), m_allowedBadPointsPercentage(0.), m_area(), m_direction_list(), m_ip_edges_list(),
  m_compute_moment(false), m_graphics(false), m_thickness(1), m_bbox_u_min(0), m_bbox_u_max(0), m_bbox_v_min(0), m_bbox_v_max(0),
  m_firstBorder_u(0), m_firstBorder_v()
{
  const unsigned int val_max = 255;
  const unsigned int val_median = 128;
  m_gray_level_min = val_median;
  m_gray_level_max = val_max;
  *this = twinDot;
}

/*!
  = operator.
*/
vpDot2 &vpDot2::operator=(const vpDot2 &twinDot)
{
  m_cog = twinDot.m_cog;

  m_width = twinDot.m_width;
  m_height = twinDot.m_height;
  m_surface = twinDot.m_surface;
  m_gray_level_min = twinDot.m_gray_level_min;
  m_gray_level_max = twinDot.m_gray_level_max;
  m_mean_gray_level = twinDot.m_mean_gray_level;
  m_grayLevelPrecision = twinDot.m_grayLevelPrecision;
  m_gamma = twinDot.m_gamma;

  m_sizePrecision = twinDot.m_sizePrecision;
  m_ellipsoidShapePrecision = twinDot.m_ellipsoidShapePrecision;
  m_maxSizeSearchDistPrecision = twinDot.m_maxSizeSearchDistPrecision;
  m_allowedBadPointsPercentage = twinDot.m_allowedBadPointsPercentage;
  m_area = twinDot.m_area;

  m_direction_list = twinDot.m_direction_list;
  m_ip_edges_list = twinDot.m_ip_edges_list;

  m_compute_moment = twinDot.m_compute_moment;
  m_graphics = twinDot.m_graphics;
  m_thickness = twinDot.m_thickness;

  m_bbox_u_min = twinDot.m_bbox_u_min;
  m_bbox_u_max = twinDot.m_bbox_u_max;
  m_bbox_v_min = twinDot.m_bbox_v_min;
  m_bbox_v_max = twinDot.m_bbox_v_max;

  m_firstBorder_u = twinDot.m_firstBorder_u;
  m_firstBorder_v = twinDot.m_firstBorder_v;

  m00 = twinDot.m00;
  m01 = twinDot.m01;
  m11 = twinDot.m11;
  m10 = twinDot.m10;
  m02 = twinDot.m02;
  m20 = twinDot.m20;

  mu11 = twinDot.mu11;
  mu20 = twinDot.mu20;
  mu02 = twinDot.mu02;

  return (*this);
}

/******************************************************************************
 *
 *      PUBLIC METHODS
 *****************************************************************************/

/*!
  Display in overlay the dot edges and center of gravity.

  \param I : Image.
  \param color : The color used for the display.
  \param t : Thickness of the displayed cross located at the dot cog.
*/
void vpDot2::display(const vpImage<unsigned char> &I, vpColor color, unsigned int t) const
{
  const unsigned int val_3 = 3;
  const unsigned int val_8 = 8;
  vpDisplay::displayCross(I, m_cog, (val_3 * t) + val_8, color, t);
  std::list<vpImagePoint>::const_iterator it;

  std::list<vpImagePoint>::const_iterator ip_edges_list_end = m_ip_edges_list.end();
  for (it = m_ip_edges_list.begin(); it != ip_edges_list_end; ++it) {
    vpDisplay::displayPoint(I, *it, color);
  }
}

/*!

  Initialize the tracking with a mouse click on the image and update the dot
  characteristics (center of gravity, moments) by a call to track().

  Wait a user click in a white area in the image. The clicked pixel
  will be the starting point from which the dot will be tracked.

  To get center of gravity of the dot, see getCog(). To compute the
  moments see setComputeMoments(). To get the width or height of the
  dot, call getWidth() and getHeight(). The area of the dot is
  given by getArea().

  \param I : Image.
  \param size : Size of the dot to track.

  If no valid dot was found in the window, return an exception.

  \exception vpTrackingException::featureLostError : If the dot initialization
  failed. The initialization can fail if the following characteristics are
  not valid;
  - The gray level is between gray level min and gray level max.

  - The shape should be ellipsoid if
    setEllipsoidShapePrecision(ellipsoidShapePrecision) is used.
    This is the default case. To track a non ellipsoid shape use
    setEllipsoidShapePrecision(0).

  \sa track()

*/
void vpDot2::initTracking(const vpImage<unsigned char> &I, unsigned int size)
{
  while (vpDisplay::getClick(I, m_cog) != true) {
    // block empty waiting user interaction
  }

  unsigned int i = static_cast<unsigned int>(m_cog.get_i());
  unsigned int j = static_cast<unsigned int>(m_cog.get_j());
  const unsigned int val_max = 255;

  double Ip = pow(static_cast<double>(I[i][j]) / val_max, 1 / m_gamma);

  if ((Ip - (1 - m_grayLevelPrecision)) < 0) {
    m_gray_level_min = 0;
  }
  else {
    m_gray_level_min = static_cast<unsigned int>(val_max * pow(Ip - (1 - m_grayLevelPrecision), m_gamma));
    if (m_gray_level_min > val_max) {
      m_gray_level_min = val_max;
    }
  }
  m_gray_level_max = static_cast<unsigned int>(val_max * pow(Ip + (1 - m_grayLevelPrecision), m_gamma));
  if (m_gray_level_max > val_max) {
    m_gray_level_max = val_max;
  }

  setWidth(size);
  setHeight(size);

  track(I);
}

/*!

  Initialize the tracking for a dot supposed to be located at (u,v) and update
  the dot characteristics (center of gravity, moments) by a call to track().

  \param I : Image to process.

  \param ip : Location of the starting point from which the dot will be
  tracked in the image.

  \param size : Size of the dot to track.

  To get center of gravity of the dot, see getCog(). To compute the
  moments see setComputeMoments().

  If no valid dot was found in the window, return an exception.

  \exception vpTrackingException::featureLostError : If the dot initialization
  failed. The initialization can fail if the following characteristics are
  not valid;
  - The gray level is between gray level min and gray level max.

  - The shape should be ellipsoid if
    setEllipsoidShapePrecision(ellipsoidShapePrecision) is used.
    This is the default case. To track a non ellipsoid shape use
    setEllipsoidShapePrecision(0).
*/
void vpDot2::initTracking(const vpImage<unsigned char> &I, const vpImagePoint &ip, unsigned int size)
{
  m_cog = ip;

  unsigned int i = static_cast<unsigned int>(m_cog.get_i());
  unsigned int j = static_cast<unsigned int>(m_cog.get_j());
  const unsigned int val_max = 255;

  double Ip = pow(static_cast<double>(I[i][j]) / val_max, 1 / m_gamma);

  if ((Ip - (1 - m_grayLevelPrecision)) < 0) {
    m_gray_level_min = 0;
  }
  else {
    m_gray_level_min = static_cast<unsigned int>(val_max * pow(Ip - (1 - m_grayLevelPrecision), m_gamma));
    if (m_gray_level_min > val_max) {
      m_gray_level_min = val_max;
    }
  }
  m_gray_level_max = static_cast<unsigned int>(val_max * pow(Ip + (1 - m_grayLevelPrecision), m_gamma));
  if (m_gray_level_max > val_max) {
    m_gray_level_max = val_max;
  }

  setWidth(size);
  setHeight(size);

  track(I);
}

/*!

  Initialize the tracking for a dot supposed to be located at (u,v) and update
  the dot characteristics (center of gravity, moments) by a call to track().

  The sub pixel coordinates of the dot are updated. To get the center
  of gravity coordinates of the dot, use getCog(). To
  compute the moments use setComputeMoments(true) before a call to
  initTracking().

  \param I : Image to process.

  \param ip : Location of the starting point from which the dot will
  be tracked in the image.

  \param gray_lvl_min : Minimum gray level threshold used to segment the dot;
  value comprised between 0 and 255.

  \param gray_lvl_max : Maximum gray level threshold used to segment the
  dot; value comprised between 0 and 255. \e gray_level_max should be
  greater than \e gray_level_min.

  \param size : Size of the dot to track.

  If no valid dot was found in the window, return an exception.

  \exception vpTrackingException::featureLostError : If the dot initialization
  failed. The initialization can fail if the following characteristics are
  not valid;
  - The gray level is between gray level min and gray level max.

  - The shape should be ellipsoid if
    setEllipsoidShapePrecision(ellipsoidShapePrecision) is used.
    This is the default case. To track a non ellipsoid shape use
    setEllipsoidShapePrecision(0).

  \sa track(), getCog()

*/
void vpDot2::initTracking(const vpImage<unsigned char> &I, const vpImagePoint &ip, unsigned int gray_lvl_min,
                          unsigned int gray_lvl_max, unsigned int size)
{
  m_cog = ip;

  m_gray_level_min = gray_lvl_min;
  m_gray_level_max = gray_lvl_max;

  setWidth(size);
  setHeight(size);

  track(I);
}

/*!

  Try to locate the dot in the image:

  - First, estimate the new position of the dot, using its previous position.
  - Then compute the center of gravity (surface, width height) of the
    tracked entity from the Freeman chain elements.
  - If the dot is lost (estimated point too dark, too much surface
  change,...), search the dot in a window around the previous position.
  - If no valid dot was found in the window, return an exception.

  \param I : Image.

  \param canMakeTheWindowGrow: if true, the size of the searching area is
  increased if the blob is not found, otherwise it stays the same. Default
  value is true.

  \exception vpTrackingException::featureLostError : If the dot tracking
  failed. The tracking can fail if the following characteristics are not
  valid;
  - The gray level is between gray level min and gray level max.

  - The size (width or height) and the surface (in terms of number of
    pixels) should not differ to much with the previous dot.

  - The shape should be ellipsoid if
    setEllipsoidShapePrecision(ellipsoidShapePrecision) is used.
    This is the default case. To track a non ellipsoid shape use
    setEllipsoidShapePrecision(0).

  To get the center of gravity of the dot, call getCog(). To get the
  width or height of the dot, call getWidth() and getHeight(). The area of the
  dot is given by getArea().

  To compute all the inertia moments associated to the dot see
  setComputeMoments().

  To get the pixels coordinates on the dot boundary, see getList_u() and
  getList_v().

*/
void vpDot2::track(const vpImage<unsigned char> &I, bool canMakeTheWindowGrow)
{
  m00 = 0;
  m11 = 0;
  m02 = 0;
  m20 = 0;
  m10 = 0;
  m01 = 0;

  // First, we will estimate the position of the tracked point

  // Set the search area to the entire image
  setArea(I);

  // create a copy of the dot to search
  // This copy can be saw as the previous dot used to check if the current one
  // found with computeParameters() is similar to the previous one (see
  // isValid() function). If the found dot is not similar (or valid), we use
  // this copy to set the current found dot to the previous one (see below).
  vpDot2 wantedDot(*this);

  bool found = computeParameters(I, m_cog.get_u(), m_cog.get_v());

  if (found) {
    // test if the found dot is valid (ie similar to the previous one)
    found = isValid(I, wantedDot);
    if (!found) {
      *this = wantedDot;
      // std::cout << "The found dot is not valid" << std::endl;
    }
  }

  if (!found) {
    // if estimation was wrong (get an error tracking), look for the dot
    // closest from the estimation,
    // i.e. search for dots in an a region of interest around the this dot and
    // get the first element in the area.

    // first get the size of the search window from the dot size
    double searchWindowWidth = 0.0;
    double searchWindowHeight = 0.0;

    if ((std::fabs(getWidth()) <= std::numeric_limits<double>::epsilon()) ||
        (std::fabs(getHeight()) <= std::numeric_limits<double>::epsilon())) {
      searchWindowWidth = 80.;
      searchWindowHeight = 80.;
    }
    else if (canMakeTheWindowGrow) {
      const unsigned int val_5 = 5;
      searchWindowWidth = getWidth() * val_5;
      searchWindowHeight = getHeight() * val_5;
    }
    else {
      searchWindowWidth = getWidth();
      searchWindowHeight = getHeight();
    }

    std::list<vpDot2> candidates;
    searchDotsInArea(I, static_cast<int>(m_cog.get_u() - (searchWindowWidth / 2.0)),
                     static_cast<int>(m_cog.get_v() - (searchWindowHeight / 2.0)),
                     static_cast<unsigned int>(searchWindowWidth),
                     static_cast<unsigned int>(searchWindowHeight), candidates);

    // if the vector is empty, that mean we didn't find any candidate
    // in the area, return an error tracking.
    if (candidates.empty()) {
      throw(vpTrackingException(vpTrackingException::featureLostError, "No dot was found"));
    }

    // otherwise we've got our dot, update this dot's parameters
    vpDot2 movingDot = candidates.front();

    setCog(movingDot.getCog());
    setArea(movingDot.getArea());
    setWidth(movingDot.getWidth());
    setHeight(movingDot.getHeight());

    // Update the moments
    m00 = movingDot.m00;
    m01 = movingDot.m01;
    m10 = movingDot.m10;
    m11 = movingDot.m11;
    m20 = movingDot.m20;
    m02 = movingDot.m02;

    // Update the bounding box
    m_bbox_u_min = movingDot.m_bbox_u_min;
    m_bbox_u_max = movingDot.m_bbox_u_max;
    m_bbox_v_min = movingDot.m_bbox_v_min;
    m_bbox_v_max = movingDot.m_bbox_v_max;
  }

  // if this dot is partially out of the image, return an error tracking.
  if (!isInImage(I)) {
    throw(vpTrackingException(vpTrackingException::featureLostError,
                              "The center of gravity of the dot is not in the image"));
  }

  const unsigned int val_max = 255;
  double Ip = pow(getMeanGrayLevel() / val_max, 1 / m_gamma);
  // printf("current value of gray level center : %i\n", I[v][u]);

  // get Mean Gray Level of I
  if ((Ip - (1 - m_grayLevelPrecision)) < 0) {
    m_gray_level_min = 0;
  }
  else {
    m_gray_level_min = static_cast<unsigned int>(val_max * pow(Ip - (1 - m_grayLevelPrecision), m_gamma));
    if (m_gray_level_min > val_max) {
      m_gray_level_min = val_max;
    }
  }
  m_gray_level_max = static_cast<unsigned int>(val_max * pow(Ip + (1 - m_grayLevelPrecision), m_gamma));
  if (m_gray_level_max > val_max) {
    m_gray_level_max = val_max;
  }

  if (m_graphics) {
    // display a red cross at the center of gravity's location in the image.
    const unsigned int val_3 = 3;
    const unsigned int val_8 = 8;
    vpDisplay::displayCross(I, m_cog, (val_3 * m_thickness) + val_8, vpColor::red, m_thickness);
  }
}

/*!

  Track and get the new dot coordinates. See track() for a more complete
  description

  \param[in] I : Image to process.

  \param[out] ip : Sub pixel coordinate of the tracked dot center of gravity.

  \param[in] canMakeTheWindowGrow : if true, the size of the searching area is
  increased if the blob is not found, otherwise it stays the same. Default
  value is true.

  The behavior of this method is similar to the following code:
  \code
  vpDot2 d;
  d.track(I);
  vpImagePoint cog = d.getCog();
  \endcode

  \sa track()
*/
void vpDot2::track(const vpImage<unsigned char> &I, vpImagePoint &ip, bool canMakeTheWindowGrow)
{
  track(I, canMakeTheWindowGrow);

  ip = m_cog;
}

///// GET METHODS
////////////////////////////////////////////////////////////////

/*!
  Return the width of the dot.

  \sa getHeight()
*/
double vpDot2::getWidth() const { return m_width; }

/*!
  Return the height of the dot.

  \sa getWidth()
*/
double vpDot2::getHeight() const { return m_height; }

/*!
  Return the area of the dot.

  The area of the dot is also given by \f$|m00|\f$.
*/
double vpDot2::getArea() const { return fabs(m_surface); }

/*!
  Return the precision of the gray level of the dot. It is a double
  precision float which value is in [0,1]. 1 means full precision, whereas
  values close to 0 show a very bad precision.
*/
double vpDot2::getGrayLevelPrecision() const { return m_grayLevelPrecision; }

/*!
  Return the precision of the size of the dot. It is a double
  precision float which value is in [0.05,1]. 1 means full precision, whereas
  values close to 0 show a very bad precision.
*/
double vpDot2::getSizePrecision() const { return m_sizePrecision; }

/*!
  Return the precision of the ellipsoid shape of the dot. It is a double
  precision float which value is in [0,1]. 1 means full precision, whereas
  values close to 0 show a very bad precision.

  \sa setEllipsoidShapePrecision()
*/
double vpDot2::getEllipsoidShapePrecision() const { return m_ellipsoidShapePrecision; }

/*!
  Return the precision of the search maximum distance to get the starting
  point on a dot border. It is a double precision float which value is in
  [0.05,1]. 1 means full precision, whereas values close to 0 show a very bad
  precision.
*/
double vpDot2::getMaxSizeSearchDistPrecision() const { return m_maxSizeSearchDistPrecision; }

/*!
  Return the distance between the two center of dots.
*/
double vpDot2::getDistance(const vpDot2 &distantDot) const
{
  vpImagePoint cogDistantDot = distantDot.getCog();
  double diff_u = m_cog.get_u() - cogDistantDot.get_u();
  double diff_v = m_cog.get_v() - cogDistantDot.get_v();
  return sqrt((diff_u * diff_u) + (diff_v * diff_v));
}

///// SET METHODS ////////////////////////////////////////////////////////////

/*!

  Set the width of the dot. This is meant to be used to search a dot in an
  area.

  \param w : Width of a dot to search in a region of interest.

  \sa setHeight(), setArea(), setSizePrecision()
*/
void vpDot2::setWidth(const double &w) { m_width = w; }

/*!

  Set the height of the dot. This is meant to be used to search a dot in an
  area.

  \param h : Height of a dot to search in a region of interest.

  \sa setWidth(), setArea(), setSizePrecision()

*/
void vpDot2::setHeight(const double &h) { m_height = h; }

/*!

  Set the area of the dot. This is meant to be used to search a dot in a
  region of interest.

  \param a : Area of a dot to search in a region of interest.

  \sa setWidth(), setHeight(), setSizePrecision()

*/
void vpDot2::setArea(const double &a) { m_surface = a; }

/*!

  Set the precision of the gray level of the dot.

  \param precision : It is a double precision float which value is in [0.05,1]:
  - 1 means full precision, whereas values close to 0 show a very bad
  accuracy.
  - Values lower or equal to 0.05 are brought back to 0.05.
  - Values higher than  1 are brought back to 1
  If the initial gray level is I, the gray levels of the dot will be between :
  \f$Imin=255*\big((\frac{I}{255})^{{\gamma}^{-1}}-(1-grayLevelPrecision)\big)^{\gamma}\f$
  and
  \f$Imax=255*\big((\frac{I}{255})^{{\gamma}^{-1}}+(1-grayLevelPrecision)\big)^{\gamma}\f$
  with \f$\gamma=1.5\f$ .

  \sa setGrayLevelMin(), setGrayLevelMax()
*/
void vpDot2::setGrayLevelPrecision(const double &precision)
{
  double epsilon = 0.05;
  if (m_grayLevelPrecision < epsilon) {
    m_grayLevelPrecision = epsilon;
  }
  else if (m_grayLevelPrecision > 1) {
    m_grayLevelPrecision = 1.0;
  }
  else {
    m_grayLevelPrecision = precision;
  }
}
/*!

  Set the precision of the size of the dot. Used to test the validity of the
  dot

  \param precision : It is a double precision float which value is in [0,1]:
  - this is the limit ratio between the tested parameter and the measured one.
    minSize = sizePrecision*originalSize ; maxSize =
  originalSize/sizePrecision ;
  - 1 means full precision, whereas values close to 0 show a very bad
  accuracy.
  - Values lower or equal to 0 are brought back to 0.
  - Values higher than 1 are brought back to 1.
  - To desactivate validity test set sizePrecision to 0

  \sa setWidth(), setHeight(), setArea()
*/
void vpDot2::setSizePrecision(const double &precision)
{
  if (m_sizePrecision < 0) {
    m_sizePrecision = 0;
  }
  else if (m_sizePrecision > 1) {
    m_sizePrecision = 1.0;
  }
  else {
    m_sizePrecision = precision;
  }
}

/*!
  Indicates if the dot should have an ellipsoid shape to be valid.
  \param precision : It is a double precision float which value is in [0,1]:

  - 1 means full precision, whereas values close to 0 show a very bad
  accuracy.
  - Values lower or equal to 0 are brought back to 0.
  - Values higher than  1 are brought back to 1.
  To track a non ellipsoid shape use setEllipsoidShapePrecision(0).

  The following example show how to track a blob with a height constraint on
  an ellipsoid shape. The tracking will fail if the shape is not ellipsoid.
  \code
  vpDot2 dot;
  dot.setEllipsoidShapePrecision(0.9); // to track a blob with a height
  constraint attendee on a circle shape
  ...
  dot.track();
  \endcode

  This other example shows how to remove any constraint on the shape. Here the
  tracker will be able to track any shape, including square or rectangular
  shapes.
  \code
  vpDot2 dot;
  dot.setEllipsoidShapePrecision(0.); // to track a blob without any constraint on the shape
  ...
  dot.track();
  \endcode

  \sa getEllipsoidShapePrecision()
*/
void vpDot2::setEllipsoidShapePrecision(const double &precision)
{

  if (m_ellipsoidShapePrecision < 0) {
    m_ellipsoidShapePrecision = 0;
  }
  else if (m_ellipsoidShapePrecision > 1) {
    m_ellipsoidShapePrecision = 1.0;
  }
  else {
    m_ellipsoidShapePrecision = precision;
  }
}

/*!

  Set the precision of the search maximum distance to get the starting point
  on a dot border. A too low value mean a large search area.

  \param precision : It is a double precision float which value is in
  [0.05,1]:
  - this is the limit ratio between the tested parameter and the measured one.
     distance < getWidth()/(getSizePrecision()+epsilon);
  - 1 means full precision, whereas values close to 0 show a very bad
  accuracy.
  - Values lower or equal to 0.05 are brought back to 0.05
  - Values higher than 1 are brought back to 1.

*/
void vpDot2::setMaxSizeSearchDistPrecision(const double &precision)
{
  double epsilon = 0.05;
  if (m_maxSizeSearchDistPrecision < epsilon) {
    m_maxSizeSearchDistPrecision = epsilon;
  }
  else if (m_maxSizeSearchDistPrecision > 1) {
    m_maxSizeSearchDistPrecision = 1.0;
  }
  else {
    m_maxSizeSearchDistPrecision = precision;
  }
}

/*!

  Set the parameters of the area in which a dot is search to the image
  dimension.

  \param I : Image.

*/
void vpDot2::setArea(const vpImage<unsigned char> &I) { setArea(I, 0, 0, I.getWidth(), I.getHeight()); }

/*!

  Set the parameters of an area by setting the upper-left corner coordinates
  (u, v), width and height.

  \param I : The image we are working with.
  \param u : Area horizontal left coordinate.
  \param v : Area vertical top coordinate.
  \param w : Area width.
  \param h : Area height.

*/
void vpDot2::setArea(const vpImage<unsigned char> &I, int u, int v, unsigned int w, unsigned int h)
{
  unsigned int image_w = I.getWidth();
  unsigned int image_h = I.getHeight();

  // Bounds the area to the image
  if (u < 0) {
    u = 0;
  }
  else if (u >= static_cast<int>(image_w)) {
    u = static_cast<int>(image_w) - 1;
  }
  if (v < 0) {
    v = 0;
  }
  else if (v >= static_cast<int>(image_h)) {
    v = static_cast<int>(image_h) - 1;
  }

  if ((static_cast<unsigned int>(u) + w) > image_w) {
    w = image_w - static_cast<unsigned int>(u) - 1;
  }
  if ((static_cast<unsigned int>(v) + h) > image_h) {
    h = image_h - static_cast<unsigned int>(v) - 1;
  }

  m_area.setRect(u, v, w, h);
}

/*!

  Set the parameters of the area.

  \param a : Area.

*/
void vpDot2::setArea(const vpRect &area) { m_area = area; }

///// CLASS FUNCTIONALITY ////////////////////////////////////////////////////

/*!

  Look for a list of dot matching this dot parameters within the entire
  image.

  \warning Allocates memory for the list of dots returned by this method.
  Desallocation has to be done by yourself.

  \param I : Image.

  \param niceDots: List of the dots that are found.

  Before calling this method, dot characteristics to found have to be set
  like:

  \code
  vpDot2 d;

  // Set dot characteristics for the auto detection
  d.setWidth(24);
  d.setHeight(23);
  d.setArea(412);
  d.setGrayLevelMin(160);
  d.setGrayLevelMax(255);
  d.setGrayLevelPrecision(0.8);
  d.setSizePrecision(0.65);
  d.setEllipsoidShapePrecision(0.65);
  \endcode

  To search dots in the whole image:
  \code
  std::list<vpDot2> list_d;
  d.searchDotsInArea(I, 0, 0, I.getWidth(), I.getHeight(), list_d) ;
  \endcode

  The number of dots found in the area is given by:
  \code
  std::cout << list_d.size();
  \endcode

  To parse all the dots:
  \code
  std::list<vpDot2>::iterator it;
  for (it = list_d.begin(); it != list_d.end(); ++it) {
      vpDot2 tmp_d = *it;
  }
  \endcode

  \sa searchDotsInArea(vpImage<unsigned char>&, int, int, unsigned int,
  unsigned int, std::list<vpDot2> &)
*/
void vpDot2::searchDotsInArea(const vpImage<unsigned char> &I, std::list<vpDot2> &niceDots)
{
  searchDotsInArea(I, 0, 0, I.getWidth(), I.getHeight(), niceDots);
}

/*!

  Check if the dot is "like" the wanted dot passed in.

  Compare the following characteristics of the dot to the wanted dot;
  - the size (width or height)
  - the surface (number of pixels)
  - the geometry; the shape should be ellipsoid if
    setEllipsoidShapePrecision(double ellispoidShapePrecision) is used.

  \return If it is so, return true, otherwise return false.

  \warning Parameters of the wanted dot (width, height, surface, in level, out
  level, accuracy) must already be set before; see
  searchDotsInArea(vpImage<unsigned char>& I)

  \param I : Image.
  \param wantedDot : Wanted dot passed in.

*/
bool vpDot2::isValid(const vpImage<unsigned char> &I, const vpDot2 &wantedDot)
{
  double size_precision = wantedDot.getSizePrecision();
  double ellipsoidShape_precision = wantedDot.getEllipsoidShapePrecision();

  //
  // First, check the width, height and surface of the dot. Those parameters
  // must be the same.
  //
  // if (   (wantedDot.getWidth()   != 0)
  //  && (wantedDot.getHeight()  != 0)
  //  && (wantedDot.getArea() != 0) )
  if ((std::fabs(wantedDot.getWidth()) > std::numeric_limits<double>::epsilon()) &&
      (std::fabs(wantedDot.getHeight()) > std::numeric_limits<double>::epsilon()) &&
      (std::fabs(wantedDot.getArea()) > std::numeric_limits<double>::epsilon())) {
    if (std::fabs(size_precision) > std::numeric_limits<double>::epsilon()) {
      double epsilon = 0.001;
#ifdef DEBUG
      std::cout << "test size precision......................\n";
      std::cout << "wanted dot: "
        << "w=" << wantedDot.getWidth() << " h=" << wantedDot.getHeight() << " s=" << wantedDot.getArea()
        << " precision=" << size_precision << " epsilon=" << epsilon << std::endl;
      std::cout << "dot found: "
        << "w=" << getWidth() << " h=" << getHeight() << " s=" << getArea() << std::endl;
#endif

      if ((((wantedDot.getWidth() * size_precision) - epsilon) < getWidth()) == false) {
#ifdef DEBUG
        printf("Bad width > for dot (%g, %g)\n", m_cog.get_u(), m_cog.get_v());
#endif
        return false;
      }

      if ((getWidth() < (wantedDot.getWidth() / (size_precision + epsilon))) == false) {
#ifdef DEBUG
        printf("Bad width %g > %g for dot (%g, %g)\n", getWidth(), wantedDot.getWidth() / (size_precision + epsilon),
               m_cog.get_u(), m_cog.get_v());
#endif
        return false;
      }

      if ((((wantedDot.getHeight() * size_precision) - epsilon) < getHeight()) == false) {
#ifdef DEBUG
        printf("Bad height %g > %g for dot (%g, %g)\n", wantedDot.getHeight() * size_precision - epsilon, getHeight(),
               m_cog.get_u(), m_cog.get_v());
#endif
        return false;
      }

      if ((getHeight() < (wantedDot.getHeight() / (size_precision + epsilon))) == false) {
#ifdef DEBUG
        printf("Bad height %g > %g for dot (%g, %g)\n", getHeight(), wantedDot.getHeight() / (size_precision + epsilon),
               m_cog.get_u(), m_cog.get_v());
#endif
        return false;
      }

      if ((((wantedDot.getArea() * (size_precision * size_precision)) - epsilon) < getArea()) == false) {
#ifdef DEBUG
        printf("Bad surface %g > %g for dot (%g, %g)\n",
               wantedDot.getArea() * (size_precision * size_precision) - epsilon, getArea(), m_cog.get_u(), m_cog.get_v());
#endif
        return false;
      }

      if ((getArea() < (wantedDot.getArea() / ((size_precision * size_precision) + epsilon))) == false) {
#ifdef DEBUG
        printf("Bad surface %g < %g for dot (%g, %g)\n", getArea(),
               wantedDot.getArea() / (size_precision * size_precision + epsilon), m_cog.get_u(), m_cog.get_v());
#endif
        return false;
      }
    }
  }
  //
  // Now we can proceed to more advanced (and costy) checks.
  // First check there is a white (>level) elipse within dot
  // Then check the dot is surrounded by a black ellipse.
  //
  int nb_point_to_test = 20; // Nb points to test on inner and outside ellipsoid
  int nb_bad_points = 0;
  int nb_max_bad_points = static_cast<int>(nb_point_to_test * m_allowedBadPointsPercentage);
  double step_angle = (2 * M_PI) / nb_point_to_test;

  // --comment: if ellipsoidShape_precision diff 0 and compute_moment is true
  if ((std::fabs(ellipsoidShape_precision) > std::numeric_limits<double>::epsilon()) && m_compute_moment) {
    // Chaumette, Image Moments: A General and Useful Set of Features for Visual Servoing, TRO 2004, eq 15

    /*
    // -comment: mu11 = m11 - m00 * xg * yg = m11 - m00 * m10/m00 * m01/m00
    // -comment:      = m11 - m10 * m01 / m00
    // -comment: mu20 = m20 - m00 * xg^2 = m20 - m00 * m10/m00 * m10/m00
    // -comment:      = m20 - m10^2 / m00
    // -comment: mu02 = m02 - m01^2 / m00
    // -comment: alpha = 1/2 arctan( 2 * mu11 / (mu20 - mu02) )
    //
    // -comment: a1^2 = 2 / m00 * (mu02 + mu20 + sqrt( (mu20 - mu02)^2 + 4mu11^2) )
    //
    // -comment: a2^2 = 2 / m00 * (mu02 + mu20 - sqrt( (mu20 - mu02)^2 + 4mu11^2) )
    */
    // we compute parameters of the estimated ellipse
    double tmp1 = (((m01 * m01) - (m10 * m10)) / m00) + (m20 - m02);
    double tmp2 = m11 - ((m10 * m01) / m00);
    double Sqrt = sqrt((tmp1 * tmp1) + (4 * tmp2 * tmp2));
    double a1 = sqrt((2 / m00) * (((m20 + m02) - (((m10 * m10) + (m01 * m01)) / m00)) + Sqrt));
    double a2 = sqrt((2 / m00) * (((m20 + m02) - (((m10 * m10) + (m01 * m01)) / m00)) - Sqrt));
    double alpha = 0.5 * atan2(2 * ((m11 * m00) - (m10 * m01)), ((((m20 - m02) * m00) - (m10 * m10)) + (m01 * m01)));

    // to be able to track small dots, minorize the ellipsoid radius for the
    // inner test
    a1 -= 1.0;
    a2 -= 1.0;

    double innerCoef = ellipsoidShape_precision;
    unsigned int u, v;
    double cog_u = m_cog.get_u();
    double cog_v = m_cog.get_v();
    double val_2 = 2;

    vpImagePoint ip;
    nb_bad_points = 0;
    for (double theta = 0.; theta < (val_2 * M_PI); theta += step_angle) {
      u = static_cast<unsigned int>(cog_u + (innerCoef * ((a1 * cos(alpha) * cos(theta)) - (a2 * sin(alpha) * sin(theta)))));
      v = static_cast<unsigned int>(cog_v + (innerCoef * ((a1 * sin(alpha) * cos(theta)) + (a2 * cos(alpha) * sin(theta)))));
      if (!this->hasGoodLevel(I, u, v)) {
#ifdef DEBUG
        printf("Inner circle pixel (%u, %u) has bad level for dot (%g, %g): "
               "%d not in [%u, %u]\n",
               u, v, cog_u, cog_v, I[v][u], m_gray_level_min, m_gray_level_max);
#endif
        ++nb_bad_points;
      }
      if (m_graphics) {
        for (unsigned int t = 0; t < m_thickness; ++t) {
          ip.set_u(u + t);
          ip.set_v(v);
          vpDisplay::displayPoint(I, ip, vpColor::green);
        }
      }
#ifdef DEBUG
      vpDisplay::displayPoint(I, ip, vpColor::green);
      vpDisplay::flush(I);
#endif
    }
    if (nb_bad_points > nb_max_bad_points) {
#ifdef DEBUG
      printf("Inner ellipse has %d bad points. Max allowed is %d\n", nb_bad_points, nb_max_bad_points);
#endif
      return false;
    }
    // to be able to track small dots, maximize the ellipsoid radius for the
    // inner test
    a1 += 2.0;
    a2 += 2.0;

    double outCoef = 2 - ellipsoidShape_precision; // --comment: 1.6
    nb_bad_points = 0;
    for (double theta = 0.; theta < (val_2 * M_PI); theta += step_angle) {
      u = static_cast<unsigned int>(cog_u + (outCoef * ((a1 * cos(alpha) * cos(theta)) - (a2 * sin(alpha) * sin(theta)))));
      v = static_cast<unsigned int>(cog_v + (outCoef * ((a1 * sin(alpha) * cos(theta)) + (a2 * cos(alpha) * sin(theta)))));
#ifdef DEBUG
      // vpDisplay::displayRectangle(I, area, vpColor::yellow);
      vpDisplay::displayCross(I, (int)v, (int)u, 7, vpColor::purple);
      vpDisplay::flush(I);
#endif
      // If outside the area, continue
      if ((static_cast<double>(u) < m_area.getLeft()) ||
          (static_cast<double>(u) > m_area.getRight()) ||
          (static_cast<double>(v) < m_area.getTop()) ||
          (static_cast<double>(v) > m_area.getBottom())) {
        // continue
      }
      else {
        if (!this->hasReverseLevel(I, u, v)) {
#ifdef DEBUG
          printf("Outside circle pixel (%u, %u) has bad level for dot (%g, "
                 "%g): %d not in [%u, %u]\n",
                 u, v, cog_u, cog_v, I[v][u], m_gray_level_min, m_gray_level_max);
#endif
          ++nb_bad_points;
        }
        if (m_graphics) {
          for (unsigned int t = 0; t < m_thickness; ++t) {
            ip.set_u(u + t);
            ip.set_v(v);

            vpDisplay::displayPoint(I, ip, vpColor::green);
          }
        }
      }
    }
  }
  if (nb_bad_points > nb_max_bad_points) {
#ifdef DEBUG
    printf("Outside ellipse has %d bad points. Max allowed is %d\n", nb_bad_points, nb_max_bad_points);
#endif
    return false;
  }

  return true;
}

/*!

  Check if a the pixel of coordinates (u, v) is in the image and has
  a good level to belong to this dot.

  \param I : Image.
  \param u : Pixel to test.
  \param v : Pixel to test.

  \return true : If the pixel of coordinates (u, v) is in the area and
  has a value between the min and max levels fixed by setGrayLevelMin() and
  setGrayLevelMax().

  \return false : Otherwise

  \sa setGrayLevelMin(), setGrayLevelMax()

*/
bool vpDot2::hasGoodLevel(const vpImage<unsigned char> &I, const unsigned int &u, const unsigned int &v) const
{
  if (!isInArea(u, v)) {
    return false;
  }

  if ((I[v][u] >= m_gray_level_min) && (I[v][u] <= m_gray_level_max)) {
    return true;
  }
  else {
    return false;
  }
}

/*!

  Check if a the pixel of coordinates (u, v) in the image has a good level to
  be a dark zone around the dot.

  \param I : Image.
  \param u : Pixel to test.
  \param v : Pixel to test.

  \return true if it is so, and false otherwise.

*/
bool vpDot2::hasReverseLevel(const vpImage<unsigned char> &I, const unsigned int &u, const unsigned int &v) const
{

  if (!isInArea(u, v)) {
    return false;
  }

  if ((I[v][u] < m_gray_level_min) || (I[v][u] > m_gray_level_max)) {
    return true;
  }
  else {
    return false;
  }
}

/*!
  Return a new instance of vpDot2.

  Should be used by child classed to return their own instance of vpDot2.

  \return An instance of vpDot2.

*/
vpDot2 *vpDot2::getInstance() { return new vpDot2(); }


/******************************************************************************
 *
 *      PRIVATE METHODS
 *
 ******************************************************************************/

/*!

  Compute all the parameters of the dot (center, width, height, surface,
  inertia moments...).

  This is done the following way:

  - First, we check the point (_u, _v) passed in has the right level in the
    image

  - Then we cross the tracked entity from left to right until we reach it's
    border.

  - We follow this border until we come back to the first point or we get to
    border of the image. Each time we update variables used to compute the
    dot parameters

  \param I : The image we are working with.

  \param _u : A pixel coordinate inside the dot.

  \param _v : A pixel coordinate inside the dot.

  \return false : If a dot can't be found around pixel coordinates given as
  parameter

  \return true : If a dot was found.

  \sa getFirstBorder_u(), getFirstBorder_v()

*/
bool vpDot2::computeParameters(const vpImage<unsigned char> &I, const double &v_u, const double &v_v)
{
  m_direction_list.clear();
  m_ip_edges_list.clear();

  double est_u = v_u; // estimated
  double est_v = v_v;

  // if u has default value, set it to the actual center value
  // if( est_u == -1.0 )
  if (std::fabs(est_u + 1.0) <= (vpMath::maximum(std::fabs(est_u), 1.) * std::numeric_limits<double>::epsilon())) {
    est_u = m_cog.get_u();
  }

  // if v has default value, set it to the actual center value
  // if( est_v == -1.0 )
  if (std::fabs(est_v + 1.0) <= (vpMath::maximum(std::fabs(est_v), 1.) * std::numeric_limits<double>::epsilon())) {
    est_v = m_cog.get_v();
  }

  // if the estimated position of the dot is out of the image, not need to
  // continue, return an error tracking
  if (!isInArea(static_cast<unsigned int>(est_u), static_cast<unsigned int>(est_v))) {
    return false;
  }

  m_bbox_u_min = static_cast<int>(I.getWidth());
  m_bbox_u_max = 0;
  m_bbox_v_min = static_cast<int>(I.getHeight());
  m_bbox_v_max = 0;

  // if the first point doesn't have the right level then there's no point to
  // continue.
  if (!hasGoodLevel(I, static_cast<unsigned int>(est_u), static_cast<unsigned int>(est_v))) {
    return false;
  }

  // find the border

  if (!findFirstBorder(I, static_cast<unsigned int>(est_u), static_cast<unsigned int>(est_v), m_firstBorder_u, m_firstBorder_v)) {
    return false;
  }

  unsigned int dir = 6;

  // Determine the first element of the Freeman chain
  computeFreemanChainElement(I, m_firstBorder_u, m_firstBorder_v, dir);
  unsigned int firstDir = dir;

  // if we are now out of the image, return an error tracking
  if (!isInArea(m_firstBorder_u, m_firstBorder_v)) {
    return false;
  }

  // store the new direction and dot border coordinates.
  m_direction_list.push_back(dir);
  vpImagePoint ip;
  ip.set_u(m_firstBorder_u);
  ip.set_v(m_firstBorder_v);

  m_ip_edges_list.push_back(ip);

  int border_u = static_cast<int>(m_firstBorder_u);
  int border_v = static_cast<int>(m_firstBorder_v);
  int du, dv;
  float dS, dMu, dMv, dMuv, dMu2, dMv2;
  m00 = 0.0;
  m10 = 0.0;
  m01 = 0.0;
  m11 = 0.0;
  m20 = 0.0;
  m02 = 0.0;
  // while we didn't come back to the first point, follow the border
  do {
    // if it was asked, show the border
    if (m_graphics) {
      for (int t = 0; t < static_cast<int>(m_thickness); ++t) {
        ip.set_u(border_u + t);
        ip.set_v(border_v);

        vpDisplay::displayPoint(I, ip, vpColor::red);
      }
    }
#ifdef DEBUG
    vpDisplay::displayPoint(I, border_v, border_u, vpColor::red);
    vpDisplay::flush(I);
#endif
    // Determine the increments for the parameters
    computeFreemanParameters(border_u, border_v, dir, du, dv,
                             dS,                // surface
                             dMu, dMv,          // first order moments
                             dMuv, dMu2, dMv2); // second order moment

    // Update the parameters
    border_u += du; // Next position on the border
    border_v += dv;
    m00 += dS;  // enclosed area
    m10 += dMu; // First order moment along v axis
    m01 += dMv; // First order moment along u axis
    if (m_compute_moment) {
      m11 += dMuv; // Second order moment
      m20 += dMu2; // Second order moment along v axis
      m02 += dMv2; // Second order moment along u axis
    }
    // if we are now out of the image, return an error tracking
    if (!isInArea(static_cast<unsigned int>(border_u), static_cast<unsigned int>(border_v))) {
      // Can Occur on a single pixel dot located on the top border
      return false;
    }

    // store the new direction and dot border coordinates.

    m_direction_list.push_back(dir);

    ip.set_u(border_u);
    ip.set_v(border_v);
    m_ip_edges_list.push_back(ip);

    // update the extreme point of the dot.
    if (border_v < m_bbox_v_min) {
      m_bbox_v_min = border_v;
    }
    if (border_v > m_bbox_v_max) {
      m_bbox_v_max = border_v;
    }
    if (border_u < m_bbox_u_min) {
      m_bbox_u_min = border_u;
    }
    if (border_u > m_bbox_u_max) {
      m_bbox_u_max = border_u;
    }

    // move around the tracked entity by following the border.
    if (computeFreemanChainElement(I, static_cast<unsigned int>(border_u), static_cast<unsigned int>(border_v), dir) == false) {
      return false;
    }
  } while (((getFirstBorder_u() != static_cast<unsigned int>(border_u)) || (getFirstBorder_v() != static_cast<unsigned int>(border_v)) ||
            (firstDir != dir)) &&
           isInArea(static_cast<unsigned int>(border_u), static_cast<unsigned int>(border_v)));

#ifdef VP_DEBUG
#if VP_DEBUG_MODE == 3
  vpDisplay::flush(I);
#endif
#endif

  // if the surface is one or zero , the center of gravity wasn't properly
  // detected. Return an error tracking.
  // if( m00 == 0 || m00 == 1 )
  if ((std::fabs(m00) <= std::numeric_limits<double>::epsilon()) ||
      (std::fabs(m00 - 1.) <= (vpMath::maximum(std::fabs(m00), 1.) * std::numeric_limits<double>::epsilon()))) {
    return false;
  }
  else // compute the center
  {
    // this magic formula gives the coordinates of the center of gravity
    double tmpCenter_u = m10 / m00;
    double tmpCenter_v = m01 / m00;

    // Updates the second order centered moments
    if (m_compute_moment) {
      mu11 = m11 - (tmpCenter_u * m01);
      mu02 = m02 - (tmpCenter_v * m01);
      mu20 = m20 - (tmpCenter_u * m10);
    }

    m_cog.set_u(tmpCenter_u);
    m_cog.set_v(tmpCenter_v);
  }

  m_width = (m_bbox_u_max - m_bbox_u_min) + 1;
  m_height = (m_bbox_v_max - m_bbox_v_min) + 1;
  m_surface = m00;

  computeMeanGrayLevel(I);
  return true;
}

/*!
  Find the starting point on a dot border from an other point in the dot.
  the dot border is computed from this point.

  \param I : Image.
  \param u : The row coordinate of a pixel in the dot.
  \param v : The column coordinate of a pixel in the dot.
  \param border_u : The row coordinate of the found starting point.
  \param border_v : The column coordinate of the found starting point.

  \return false if the width of this dot was initialised and we already
  crossed the dot on more than the max possible width. Return true if success.

  \sa computeParameters()
*/
bool vpDot2::findFirstBorder(const vpImage<unsigned char> &I, const unsigned int &u, const unsigned int &v,
                             unsigned int &border_u, unsigned int &border_v)
{
  // find the border

  // NOTE:
  // from here we use int and not double. This is because we don't have
  // rounding problems and it's actually more a trouble than something else to
  // work with double when navigating around the dot.
  border_u = u;
  border_v = v;
  double epsilon = 0.001;

#ifdef DEBUG
  std::cout << "gray level: " << m_gray_level_min << " " << m_gray_level_max << std::endl;
#endif
  while (hasGoodLevel(I, border_u + 1, border_v) && (border_u < m_area.getRight()) /*I.getWidth()*/) {
    // if the width of this dot was initialised and we already crossed the dot
    // on more than the max possible width, no need to continue, return an
    // error tracking
    if ((getWidth() > 0) && ((border_u - u) > ((getWidth() / getMaxSizeSearchDistPrecision()) + epsilon))) {
      return false;
    }
#ifdef DEBUG
    vpDisplay::displayPoint(I, static_cast<int>(border_v), static_cast<int>(border_u) + 1, vpColor::green);
    vpDisplay::flush(I);
#endif

    ++border_u;
  }
  return true;
}

/*!

  Test if a pixel is in the image. Points of the border are not
  considered to be in the image.  Call the isInImage( vpImage<unsigned
  char> &I, const vpImagePoint &) method.

  \param I : The image.

  \return true if the pixel of coordinates (posI, posJ) is in the image and
  false otherwise.
*/
bool vpDot2::isInImage(const vpImage<unsigned char> &I) const { return isInImage(I, m_cog); }

/*!

  Test if a pixel is in the image. Points of the border are not considered to
  be in the image.

  \param I : The image.
  \param ip : An image point.

  \return true if the image point \e ip is in the image and false
  otherwise.
*/
bool vpDot2::isInImage(const vpImage<unsigned char> &I, const vpImagePoint &ip) const
{
  unsigned int h = I.getHeight();
  unsigned int w = I.getWidth();
  double u = ip.get_u();
  double v = ip.get_v();

  if ((u < 0) || (u >= w)) {
    return false;
  }
  if ((v < 0) || (v >= h)) {
    return false;
  }
  return true;
}

/*!

  Test if a pixel is in a region of interest. Points of the border are not
  considered to be in the area.

  \param u : The column coordinate of the pixel.
  \param v : The row coordinate of the pixel .

  \return true if the pixel of coordinates (u, v) is in the image and false
  otherwise.
*/
bool vpDot2::isInArea(const unsigned int &u, const unsigned int &v) const
{
  unsigned int area_u_min = static_cast<unsigned int>(m_area.getLeft());
  unsigned int area_u_max = static_cast<unsigned int>(m_area.getRight());
  unsigned int area_v_min = static_cast<unsigned int>(m_area.getTop());
  unsigned int area_v_max = static_cast<unsigned int>(m_area.getBottom());

  if ((u < area_u_min) || (u > area_u_max)) {
    return false;
  }
  if ((v < area_v_min) || (v > area_v_max)) {
    return false;
  }
  return true;
}

/*!
  Get the search grid size used to found a dot in a region of interest. This
  grid is used to parse only some pixels of the search area.

  \param gridWidth : Number of pixels between to vertical lines of the grid

  \param gridHeight : Number of pixels between to horizontal lines of the grid
*/
void vpDot2::getGridSize(unsigned int &gridWidth, unsigned int &gridHeight)
{
  // first get the research grid width and height Note that
  // 1/sqrt(2)=cos(pi/4). The grid squares should be small enough to be
  // contained in the dot. We gent this here if the dot is a perfect disc.
  // More accurate criterium to define the grid should be implemented if
  // necessary
  gridWidth = static_cast<unsigned int>((getWidth() * getMaxSizeSearchDistPrecision()) / sqrt(2.));
  gridHeight = static_cast<unsigned int>((getHeight() * getMaxSizeSearchDistPrecision()) / sqrt(2.0));

  if (gridWidth == 0) {
    gridWidth = 1;
  }
  if (gridHeight == 0) {
    gridHeight = 1;
  }
}

/*!
  Compute an approximation of  mean gray level of the dot.
  We compute it by searching the mean of vertical and diagonal points
  which gray is between min and max gray level.

  \param I: The image.

  \return the mean gray level
*/
void vpDot2::computeMeanGrayLevel(const vpImage<unsigned char> &I)
{
  int cog_u = static_cast<int>(m_cog.get_u());
  int cog_v = static_cast<int>(m_cog.get_v());

  unsigned int sum_value = 0;
  unsigned int nb_pixels = 0;

  for (unsigned int i = static_cast<unsigned int>(m_bbox_u_min); i <= static_cast<unsigned int>(m_bbox_u_max); ++i) {
    unsigned int pixel_gray = static_cast<unsigned int>(I[static_cast<unsigned int>(cog_v)][i]);
    if ((pixel_gray >= getGrayLevelMin()) && (pixel_gray <= getGrayLevelMax())) {
      sum_value += pixel_gray;
      ++nb_pixels;
    }
  }
  for (unsigned int i = static_cast<unsigned int>(m_bbox_v_min); i <= static_cast<unsigned int>(m_bbox_v_max); ++i) {
    unsigned char pixel_gray = I[i][static_cast<unsigned int>(cog_u)];
    if ((pixel_gray >= getGrayLevelMin()) && (pixel_gray <= getGrayLevelMax())) {
      sum_value += pixel_gray;
      ++nb_pixels;
    }
  }
  const unsigned int nb_min_pixels = 10;
  if (nb_pixels < nb_min_pixels) { // could be good to choose the min nb points from area of dot
    // add diagonals points to have enough point
    int imin, imax;
    if ((cog_u - m_bbox_u_min) >(cog_v - m_bbox_v_min)) {
      imin = cog_v - m_bbox_v_min;
    }
    else {
      imin = cog_u - m_bbox_u_min;
    }
    if ((m_bbox_u_max - cog_u) > (m_bbox_v_max - cog_v)) {
      imax = m_bbox_v_max - cog_v;
    }
    else {
      imax = m_bbox_u_max - cog_u;
    }
    for (int i = -imin; i <= imax; ++i) {
      unsigned int pixel_gray = static_cast<unsigned int>(I[static_cast<unsigned int>(cog_v + i)][static_cast<unsigned int>(cog_u + i)]);
      if ((pixel_gray >= getGrayLevelMin()) && (pixel_gray <= getGrayLevelMax())) {
        sum_value += pixel_gray;
        ++nb_pixels;
      }
    }

    if ((cog_u - m_bbox_u_min) > (m_bbox_v_max - cog_v)) {
      imin = m_bbox_v_max - cog_v;
    }
    else {
      imin = cog_u - m_bbox_u_min;
    }
    if ((m_bbox_u_max - cog_u) > (cog_v - m_bbox_v_min)) {
      imax = cog_v - m_bbox_v_min;
    }
    else {
      imax = m_bbox_u_max - cog_u;
    }

    for (int i = -imin; i <= imax; ++i) {
      unsigned char pixel_gray = I[static_cast<unsigned int>(cog_v - i)][static_cast<unsigned int>(cog_u + i)];
      if ((pixel_gray >= getGrayLevelMin()) && (pixel_gray <= getGrayLevelMax())) {
        sum_value += pixel_gray;
        ++nb_pixels;
      }
    }
  }

  if (nb_pixels == 0) {
    // should never happen
    throw(vpTrackingException(vpTrackingException::notEnoughPointError, "No point was found"));
  }
  else {
    m_mean_gray_level = sum_value / nb_pixels;
  }
}

/*!
  Define a number of dots from a file.
  If the file does not exist, define it by clicking an image, the dots are
  then saved into the file.

  If the dots from the file cannot be tracked in the image, will ask to click
  them.

  \param dot : dot2 array
  \param n : number of dots, array dimension
  \param dotFile : path for the file
  \param I : image
  \param col : color to print the dots (default Blue)
  \param trackDot : if true, tracks the dots in the image, if false
  simply loads the coordinates (default true)

  \return an nx2 matrix with the coordinates of the dots
*/
vpMatrix vpDot2::defineDots(vpDot2 dot[], const unsigned int &n, const std::string &dotFile, vpImage<unsigned char> &I,
                            vpColor col, bool trackDot)
{
  vpMatrix Cogs(n, 2);
  vpImagePoint cog;
  unsigned int i;
  bool fromFile = vpIoTools::checkFilename(dotFile.c_str());
  if (fromFile) {
    vpMatrix::loadMatrix(dotFile, Cogs);
    std::cout << Cogs.getRows() << " dots loaded from file " << dotFile << std::endl;
  }

  // test number of cogs in file
  if (Cogs.getRows() < n) {
    std::cout << "Dot file has a wrong number of dots : redefining them" << std::endl;
    fromFile = false;
  }

  // read from file and tracks the dots
  if (fromFile) {
    try {
      const unsigned int cross_size = 10;
      for (i = 0; i < n; ++i) {
        cog.set_uv(Cogs[i][0], Cogs[i][1]);
        dot[i].setGraphics(true);
        dot[i].setCog(cog);
        if (trackDot) {
          dot[i].initTracking(I, cog);
          dot[i].track(I);
          vpDisplay::displayCross(I, cog, cross_size, col);
        }
      }
    }
    catch (...) {
      std::cout << "Cannot track dots from file" << std::endl;
      fromFile = false;
    }
    vpDisplay::flush(I);

    // check that dots are far away ones from the other
    i = 0;
    while ((i < n) && fromFile) {
      double d = sqrt(vpMath::sqr(dot[i].getHeight()) + vpMath::sqr(dot[i].getWidth()));
      unsigned int j = 0;
      while ((j < n) && fromFile) {
        if (j != i) {
          if (dot[i].getDistance(dot[j]) < d) {
            fromFile = false;
            std::cout << "Dots from file seem incoherent" << std::endl;
          }
        }
        ++j;
      }
      ++i;
    }
  }

  if (!fromFile) {
    vpDisplay::display(I);
    vpDisplay::flush(I);

    std::cout << "Click on the " << n << " dots clockwise starting from upper/left dot..." << std::endl;
    const unsigned int cross_size = 10;
    for (i = 0; i < n; ++i) {
      if (trackDot) {
        dot[i].setGraphics(true);
        dot[i].initTracking(I);
        cog = dot[i].getCog();
      }
      else {
        vpDisplay::getClick(I, cog);
        dot[i].setCog(cog);
      }
      Cogs[i][0] = cog.get_u();
      Cogs[i][1] = cog.get_v();
      vpDisplay::displayCross(I, cog, cross_size, col);
      vpDisplay::flush(I);
    }
  }

  if ((!fromFile) && (dotFile != "")) {
    vpMatrix::saveMatrix(dotFile, Cogs);
    std::cout << Cogs.getRows() << " dots written to file " << dotFile << std::endl;
  }

  // back to non graphic mode
  for (i = 0; i < n; ++i) {
    dot[i].setGraphics(false);
  }

  return Cogs;
}

/*!
  Tracks a number of dots in an image and displays their trajectories

  \param dot : dot2 array

  \param n : number of dots, array dimension

  \param I : image

  \param cogs : vector of vpImagePoint that will be updated with the new
  dots, will be displayed in green

  \param cogStar (optional) : array of
  vpImagePoint indicating the desired position (default nullptr), will be
  displayed in red
*/
void vpDot2::trackAndDisplay(vpDot2 dot[], const unsigned int &n, vpImage<unsigned char> &I,
                             std::vector<vpImagePoint> &cogs, vpImagePoint *cogStar)
{
  // tracking
  for (unsigned int i = 0; i < n; ++i) {
    dot[i].track(I);
    cogs.push_back(dot[i].getCog());
  }
  // trajectories
  unsigned int cogs_size = static_cast<unsigned int>(cogs.size());
  for (unsigned int i = n; i < cogs_size; ++i) {
    const unsigned int circle_size = 4;
    vpDisplay::displayCircle(I, cogs[i], circle_size, vpColor::green, true);
  }
  // initial position
  for (unsigned int i = 0; i < n; ++i) {
    const unsigned int circle_size = 4;
    vpDisplay::displayCircle(I, cogs[i], circle_size, vpColor::blue, true);
  }
  // if exists, desired position
  if (cogStar != nullptr) {
    const unsigned int circle_size = 10;
    for (unsigned int i = 0; i < n; ++i) {
      vpDisplay::displayDotLine(I, cogStar[i], dot[i].getCog(), vpColor::red);
      vpDisplay::displayCircle(I, cogStar[i], circle_size, vpColor::red, true);
    }
  }
  vpDisplay::flush(I);
}

/*!

  Display the dot center of gravity and its list of edges.

  \param I : The image used as background.

  \param cog : The center of gravity.

  \param edges_list : The list of edges;

  \param color : Color used to display the dot.

  \param thickness : Thickness of the dot.
*/
void vpDot2::display(const vpImage<unsigned char> &I, const vpImagePoint &cog,
                     const std::list<vpImagePoint> &edges_list, vpColor color, unsigned int thickness)
{
  const unsigned int val_3 = 3;
  const unsigned int val_8 = 8;

  vpDisplay::displayCross(I, cog, (val_3 * thickness) + val_8, color, thickness);
  std::list<vpImagePoint>::const_iterator it;

  std::list<vpImagePoint>::const_iterator edges_list_end = edges_list.end();
  for (it = edges_list.begin(); it != edges_list_end; ++it) {
    vpDisplay::displayPoint(I, *it, color);
  }
}

/*!

  Display the dot center of gravity and its list of edges.

  \param I : The image used as background.

  \param cog : The center of gravity.

  \param edges_list : The list of edges;

  \param color : Color used to display the dot.

  \param thickness : Thickness of the dot.
*/
void vpDot2::display(const vpImage<vpRGBa> &I, const vpImagePoint &cog, const std::list<vpImagePoint> &edges_list,
                     vpColor color, unsigned int thickness)
{
  const unsigned int val_3 = 3;
  const unsigned int val_8 = 8;
  vpDisplay::displayCross(I, cog, (val_3 * thickness) + val_8, color, thickness);
  std::list<vpImagePoint>::const_iterator it;
  std::list<vpImagePoint>::const_iterator edges_list_end = edges_list.end();
  for (it = edges_list.begin(); it != edges_list_end; ++it) {
    vpDisplay::displayPoint(I, *it, color);
  }
}

/*!
  Writes the dot center of gravity coordinates in the frame (i,j) (For more
  details about the orientation of the frame see the vpImagePoint
  documentation) to the stream \e os, and returns a reference to the stream.
*/
VISP_EXPORT std::ostream &operator<<(std::ostream &os, vpDot2 &d) { return (os << "(" << d.getCog() << ")"); }

END_VISP_NAMESPACE
