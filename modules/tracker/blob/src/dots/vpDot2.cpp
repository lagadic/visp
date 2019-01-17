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
 * Track a white dot.
 *
 * Authors:
 * Fabien Spindler
 * Anthony Saunier
 *
 *****************************************************************************/

/*!
  \file vpDot2.cpp
  \brief Track a dot.
*/

//#define DEBUG

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
  cog.set_u(0);
  cog.set_v(0);

  width = 0;
  height = 0;
  surface = 0;
  mean_gray_level = 0;
  gray_level_min = 128;
  gray_level_max = 255;
  grayLevelPrecision = 0.80;
  gamma = 1.5;

  sizePrecision = 0.65;
  ellipsoidShapePrecision = 0.65;
  maxSizeSearchDistancePrecision = 0.65;
  setEllipsoidBadPointsPercentage();
  m00 = m11 = m02 = m20 = m10 = m01 = 0.;
  mu11 = mu02 = mu20 = 0.;

  bbox_u_min = bbox_u_max = bbox_v_min = bbox_v_max = 0;

  firstBorder_u = 0;
  firstBorder_v = 0;

  compute_moment = false;
  graphics = false;
  thickness = 1;
}

/*!
  Default constructor. Just do basic default initialization.
*/
vpDot2::vpDot2()
  : m00(0.), m10(0.), m01(0.), m11(0.), m20(0.), m02(0.), mu11(0.), mu20(0.), mu02(0.), cog(), width(0), height(0),
    surface(0), gray_level_min(128), gray_level_max(255), mean_gray_level(0), grayLevelPrecision(0.8), gamma(1.5),
    sizePrecision(0.65), ellipsoidShapePrecision(0.65), maxSizeSearchDistancePrecision(0.65),
    allowedBadPointsPercentage_(0.), area(), direction_list(), ip_edges_list(), compute_moment(false), graphics(false),
    thickness(1), bbox_u_min(0), bbox_u_max(0), bbox_v_min(0), bbox_v_max(0), firstBorder_u(0), firstBorder_v()
{
}

/*!

  Constructor initialize the coordinates of the gravity center of the dot to
  the image point \e ip.  Rest is the same as the default constructor.

  \param ip : An image point with sub-pixel coordinates.

*/
vpDot2::vpDot2(const vpImagePoint &ip)
  : m00(0.), m10(0.), m01(0.), m11(0.), m20(0.), m02(0.), mu11(0.), mu20(0.), mu02(0.), cog(ip), width(0), height(0),
    surface(0), gray_level_min(128), gray_level_max(255), mean_gray_level(0), grayLevelPrecision(0.8), gamma(1.5),
    sizePrecision(0.65), ellipsoidShapePrecision(0.65), maxSizeSearchDistancePrecision(0.65),
    allowedBadPointsPercentage_(0.), area(), direction_list(), ip_edges_list(), compute_moment(false), graphics(false),
    thickness(1), bbox_u_min(0), bbox_u_max(0), bbox_v_min(0), bbox_v_max(0), firstBorder_u(0), firstBorder_v()
{
}

/*!
  Copy constructor.
*/
vpDot2::vpDot2(const vpDot2 &twinDot)
  : vpTracker(twinDot), m00(0.), m10(0.), m01(0.), m11(0.), m20(0.), m02(0.), mu11(0.), mu20(0.), mu02(0.), cog(),
    width(0), height(0), surface(0), gray_level_min(128), gray_level_max(255), mean_gray_level(0),
    grayLevelPrecision(0.8), gamma(1.5), sizePrecision(0.65), ellipsoidShapePrecision(0.65),
    maxSizeSearchDistancePrecision(0.65), allowedBadPointsPercentage_(0.), area(), direction_list(), ip_edges_list(),
    compute_moment(false), graphics(false), thickness(1), bbox_u_min(0), bbox_u_max(0), bbox_v_min(0), bbox_v_max(0),
    firstBorder_u(0), firstBorder_v()
{
  *this = twinDot;
}

/*!
  = operator.
*/
vpDot2 &vpDot2::operator=(const vpDot2 &twinDot)
{
  cog = twinDot.cog;

  width = twinDot.width;
  height = twinDot.height;
  surface = twinDot.surface;
  gray_level_min = twinDot.gray_level_min;
  gray_level_max = twinDot.gray_level_max;
  mean_gray_level = twinDot.mean_gray_level;
  grayLevelPrecision = twinDot.grayLevelPrecision;
  gamma = twinDot.gamma;
  ;
  sizePrecision = twinDot.sizePrecision;
  ellipsoidShapePrecision = twinDot.ellipsoidShapePrecision;
  maxSizeSearchDistancePrecision = twinDot.maxSizeSearchDistancePrecision;
  allowedBadPointsPercentage_ = twinDot.allowedBadPointsPercentage_;
  area = twinDot.area;

  direction_list = twinDot.direction_list;
  ip_edges_list = twinDot.ip_edges_list;

  compute_moment = twinDot.compute_moment;
  graphics = twinDot.graphics;
  thickness = twinDot.thickness;

  bbox_u_min = twinDot.bbox_u_min;
  bbox_u_max = twinDot.bbox_u_max;
  bbox_v_min = twinDot.bbox_v_min;
  bbox_v_max = twinDot.bbox_v_max;

  firstBorder_u = twinDot.firstBorder_u;
  firstBorder_v = twinDot.firstBorder_v;

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

/*!
  Destructor... do nothing for the moment.
*/
vpDot2::~vpDot2() {}

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
  vpDisplay::displayCross(I, cog, 3 * t + 8, color, t);
  std::list<vpImagePoint>::const_iterator it;

  for (it = ip_edges_list.begin(); it != ip_edges_list.end(); ++it) {
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
  while (vpDisplay::getClick(I, cog) != true) {}

  unsigned int i = (unsigned int)cog.get_i();
  unsigned int j = (unsigned int)cog.get_j();

  double Ip = pow((double)I[i][j] / 255, 1 / gamma);

  if (Ip - (1 - grayLevelPrecision) < 0) {
    gray_level_min = 0;
  } else {
    gray_level_min = (unsigned int)(255 * pow(Ip - (1 - grayLevelPrecision), gamma));
    if (gray_level_min > 255)
      gray_level_min = 255;
  }
  gray_level_max = (unsigned int)(255 * pow(Ip + (1 - grayLevelPrecision), gamma));
  if (gray_level_max > 255)
    gray_level_max = 255;

  setWidth(size);
  setHeight(size);

  try {
    track(I);
  } catch (const vpException &e) {
    // vpERROR_TRACE("Error caught") ;
    throw(e);
  }
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
  cog = ip;

  unsigned int i = (unsigned int)cog.get_i();
  unsigned int j = (unsigned int)cog.get_j();

  double Ip = pow((double)I[i][j] / 255, 1 / gamma);

  if (Ip - (1 - grayLevelPrecision) < 0) {
    gray_level_min = 0;
  } else {
    gray_level_min = (unsigned int)(255 * pow(Ip - (1 - grayLevelPrecision), gamma));
    if (gray_level_min > 255)
      gray_level_min = 255;
  }
  gray_level_max = (unsigned int)(255 * pow(Ip + (1 - grayLevelPrecision), gamma));
  if (gray_level_max > 255)
    gray_level_max = 255;

  setWidth(size);
  setHeight(size);

  try {
    track(I);
  } catch (const vpException &e) {
    // vpERROR_TRACE("Error caught") ;
    throw(e);
  }
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
  cog = ip;

  this->gray_level_min = gray_lvl_min;
  this->gray_level_max = gray_lvl_max;

  setWidth(size);
  setHeight(size);

  try {
    track(I);
  } catch (const vpException &e) {
    // vpERROR_TRACE("Error caught") ;
    throw(e);
  }
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
void vpDot2::track(const vpImage<unsigned char> &I)
{
  m00 = m11 = m02 = m20 = m10 = m01 = 0;

  // First, we will estimate the position of the tracked point

  // Set the search area to the entire image
  setArea(I);

  // create a copy of the dot to search
  // This copy can be saw as the previous dot used to check if the current one
  // found with computeParameters() is similar to the previous one (see
  // isValid() function). If the found dot is not similar (or valid), we use
  // this copy to set the current found dot to the previous one (see below).
  vpDot2 wantedDot(*this);

  //   vpDEBUG_TRACE(0, "Previous dot: ");
  //   vpDEBUG_TRACE(0, "u: %f v: %f", get_u(), get_v());
  //   vpDEBUG_TRACE(0, "w: %f h: %f", getWidth(), getHeight());
  bool found = computeParameters(I, cog.get_u(), cog.get_v());

  if (found) {
    // test if the found dot is valid (ie similar to the previous one)
    found = isValid(I, wantedDot);
    if (!found) {
      *this = wantedDot;
      // std::cout << "The found dot is not valid" << std::endl;
    }
  }

  if (!found) {
    //     vpDEBUG_TRACE(0, "Search the dot in a biggest window around the
    //     last position"); vpDEBUG_TRACE(0, "Bad computed dot: ");
    //     vpDEBUG_TRACE(0, "u: %f v: %f", get_u(), get_v());
    //     vpDEBUG_TRACE(0, "w: %f h: %f", getWidth(), getHeight());

    // if estimation was wrong (get an error tracking), look for the dot
    // closest from the estimation,
    // i.e. search for dots in an a region of interest around the this dot and
    // get the first element in the area.

    // first get the size of the search window from the dot size
    double searchWindowWidth, searchWindowHeight;
    // if( getWidth() == 0 || getHeight() == 0 )
    if (std::fabs(getWidth()) <= std::numeric_limits<double>::epsilon() ||
        std::fabs(getHeight()) <= std::numeric_limits<double>::epsilon()) {
      searchWindowWidth = 80.;
      searchWindowHeight = 80.;
    } else {
      searchWindowWidth = getWidth() * 5;
      searchWindowHeight = getHeight() * 5;
    }
    std::list<vpDot2> candidates;
    searchDotsInArea(I, (int)(this->cog.get_u() - searchWindowWidth / 2.0),
                     (int)(this->cog.get_v() - searchWindowHeight / 2.0), (unsigned int)searchWindowWidth,
                     (unsigned int)searchWindowHeight, candidates);

    // if the vector is empty, that mean we didn't find any candidate
    // in the area, return an error tracking.
    if (candidates.empty()) {
      // vpERROR_TRACE("No dot was found") ;
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
    bbox_u_min = movingDot.bbox_u_min;
    bbox_u_max = movingDot.bbox_u_max;
    bbox_v_min = movingDot.bbox_v_min;
    bbox_v_max = movingDot.bbox_v_max;
  }
  //   else {
  //     // test if the found dot is valid,
  //     if( ! isValid( I, wantedDot ) ) {
  //       *this = wantedDot;
  //       vpERROR_TRACE("The found dot is invalid:",
  // 		    "- could be a problem of size (width or height) or "
  // 		    "  surface (number of pixels) which differ too much "
  // 		    "  to the previous one "
  // 		    "- or a problem of the shape which is not ellipsoid if "
  // 		    "  use setEllipsoidShapePrecision(double
  // ellipsoidShapePrecision) "
  //         "  which is the default case. "
  // 		    "  To track a non ellipsoid shape use
  // setEllipsoidShapePrecision(0)") ;
  //       throw(vpTrackingException(vpTrackingException::featureLostError,
  // 				"The found dot is invalid")) ;
  //     }
  //   }

  // if this dot is partially out of the image, return an error tracking.
  if (!isInImage(I)) {
    // vpERROR_TRACE("The center of gravity of the dot is not in the image") ;
    throw(vpTrackingException(vpTrackingException::featureLostError,
                              "The center of gravity of the dot is not in the image"));
  }

  // Get dots center of gravity
  // unsigned int u = (unsigned int) this->cog.get_u();
  // unsigned int v = (unsigned int) this->cog.get_v();
  // Updates the min and max gray levels for the next iteration
  // double Ip = pow((double)I[v][u]/255,1/gamma);
  double Ip = pow(getMeanGrayLevel() / 255, 1 / gamma);
  // printf("current value of gray level center : %i\n", I[v][u]);

  // getMeanGrayLevel(I);
  if (Ip - (1 - grayLevelPrecision) < 0) {
    gray_level_min = 0;
  } else {
    gray_level_min = (unsigned int)(255 * pow(Ip - (1 - grayLevelPrecision), gamma));
    if (gray_level_min > 255)
      gray_level_min = 255;
  }
  gray_level_max = (unsigned int)(255 * pow(Ip + (1 - grayLevelPrecision), gamma));
  if (gray_level_max > 255)
    gray_level_max = 255;

  // printf("%i %i \n",gray_level_max,gray_level_min);
  if (graphics) {
    // display a red cross at the center of gravity's location in the image.

    vpDisplay::displayCross(I, this->cog, 3 * thickness + 8, vpColor::red, thickness);
    // vpDisplay::flush(I);
  }
}

/*!

  Track and get the new dot coordinates. See track() for a more complete
  description

  \param I : Image to process.

  \param ip [out] : Sub pixel coordinate of the tracked dot center of gravity.

  The behavior of this method is similar to the following code:
  \code
  vpDot2 d;
  d.track(I);
  vpImagePoint cog = d.getCog();
  \endcode

  \sa track()
*/
void vpDot2::track(const vpImage<unsigned char> &I, vpImagePoint &ip)
{
  track(I);

  ip = this->cog;
}

///// GET METHODS
////////////////////////////////////////////////////////////////

/*!
  Return the width of the dot.

  \sa getHeight()
*/
double vpDot2::getWidth() const { return width; }

/*!
  Return the height of the dot.

  \sa getWidth()
*/
double vpDot2::getHeight() const { return height; }

/*!
  Return the area of the dot.

  The area of the dot is also given by \f$|m00|\f$.
*/
double vpDot2::getArea() const { return fabs(surface); }

/*!
  Return the precision of the gray level of the dot. It is a double
  precision float which value is in [0,1]. 1 means full precision, whereas
  values close to 0 show a very bad precision.
*/
double vpDot2::getGrayLevelPrecision() const { return grayLevelPrecision; }

/*!
  Return the precision of the size of the dot. It is a double
  precision float which value is in [0.05,1]. 1 means full precision, whereas
  values close to 0 show a very bad precision.
*/
double vpDot2::getSizePrecision() const { return sizePrecision; }

/*!
  Return the precision of the ellipsoid shape of the dot. It is a double
  precision float which value is in [0,1]. 1 means full precision, whereas
  values close to 0 show a very bad precision.

  \sa setEllipsoidShapePrecision()
*/
double vpDot2::getEllipsoidShapePrecision() const { return ellipsoidShapePrecision; }

/*!
  Return the precision of the search maximum distance to get the starting
  point on a dot border. It is a double precision float which value is in
  [0.05,1]. 1 means full precision, whereas values close to 0 show a very bad
  precision.
*/
double vpDot2::getMaxSizeSearchDistancePrecision() const { return maxSizeSearchDistancePrecision; }

/*!
  Return the distance between the two center of dots.
*/
double vpDot2::getDistance(const vpDot2 &distantDot) const
{
  vpImagePoint cogDistantDot = distantDot.getCog();
  double diff_u = this->cog.get_u() - cogDistantDot.get_u();
  double diff_v = this->cog.get_v() - cogDistantDot.get_v();
  return sqrt(diff_u * diff_u + diff_v * diff_v);
}

///// SET METHODS ////////////////////////////////////////////////////////////

/*!

  Set the width of the dot. This is meant to be used to search a dot in an
  area.

  \param w : Width of a dot to search in a region of interest.

  \sa setHeight(), setArea(), setSizePrecision()
*/
void vpDot2::setWidth(const double &w) { this->width = w; }

/*!

  Set the height of the dot. This is meant to be used to search a dot in an
  area.

  \param h : Height of a dot to search in a region of interest.

  \sa setWidth(), setArea(), setSizePrecision()

*/
void vpDot2::setHeight(const double &h) { this->height = h; }

/*!

  Set the area of the dot. This is meant to be used to search a dot in a
  region of interest.

  \param a : Area of a dot to search in a region of interest.

  \sa setWidth(), setHeight(), setSizePrecision()

*/
void vpDot2::setArea(const double &a) { this->surface = a; }

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
  if (grayLevelPrecision < epsilon) {
    this->grayLevelPrecision = epsilon;
  } else if (grayLevelPrecision > 1) {
    this->grayLevelPrecision = 1.0;
  } else {
    this->grayLevelPrecision = precision;
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
  if (sizePrecision < 0) {
    this->sizePrecision = 0;
  } else if (sizePrecision > 1) {
    this->sizePrecision = 1.0;
  } else {
    this->sizePrecision = precision;
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

  if (ellipsoidShapePrecision < 0) {
    this->ellipsoidShapePrecision = 0;
  } else if (ellipsoidShapePrecision > 1) {
    this->ellipsoidShapePrecision = 1.0;
  } else {
    this->ellipsoidShapePrecision = precision;
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
void vpDot2::setMaxSizeSearchDistancePrecision(const double &precision)
{
  double epsilon = 0.05;
  if (maxSizeSearchDistancePrecision < epsilon) {
    this->maxSizeSearchDistancePrecision = epsilon;
  } else if (maxSizeSearchDistancePrecision > 1) {
    this->maxSizeSearchDistancePrecision = 1.0;
  } else {
    this->maxSizeSearchDistancePrecision = precision;
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
  if (u < 0)
    u = 0;
  else if (u >= (int)image_w)
    u = (int)image_w - 1;
  if (v < 0)
    v = 0;
  else if (v >= (int)image_h)
    v = (int)image_h - 1;

  if (((unsigned int)u + w) > image_w)
    w = image_w - (unsigned int)u - 1;
  if (((unsigned int)v + h) > image_h)
    h = image_h - (unsigned int)v - 1;

  area.setRect(u, v, w, h);
}

/*!

  Set the parameters of the area.

  \param a : Area.

*/
void vpDot2::setArea(const vpRect &a) { area = a; }

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

  if (graphics) {
    // Display the area were the dot is search
    vpDisplay::displayRectangle(I, area, vpColor::blue, false, thickness);
    // vpDisplay::flush(I);
  }

#ifdef DEBUG
  vpDisplay::displayRectangle(I, area, vpColor::blue);
  vpDisplay::flush(I);
#endif
  // start the search loop; for all points of the search grid,
  // test if the pixel belongs to a valid dot.
  // if it is so eventually add it to the vector of valid dots.
  std::list<vpDot2> badDotsVector;
  std::list<vpDot2>::iterator itnice;
  std::list<vpDot2>::iterator itbad;

  vpDot2 *dotToTest = NULL;
  vpDot2 tmpDot;

  unsigned int area_u_min = (unsigned int)area.getLeft();
  unsigned int area_u_max = (unsigned int)area.getRight();
  unsigned int area_v_min = (unsigned int)area.getTop();
  unsigned int area_v_max = (unsigned int)area.getBottom();

  unsigned int u, v;
  vpImagePoint cogTmpDot;

  for (v = area_v_min; v < area_v_max; v = v + gridHeight) {
    for (u = area_u_min; u < area_u_max; u = u + gridWidth) {
      // if the pixel we're in doesn't have the right color (outside the
      // graylevel interval), no need to check further, just get to the
      // next grid intersection.
      if (!hasGoodLevel(I, u, v))
        continue;

      // Test if an other germ is inside the bounding box of a dot previously
      // detected
      bool good_germ = true;

      itnice = niceDots.begin();
      while (itnice != niceDots.end() && good_germ == true) {
        tmpDot = *itnice;

        cogTmpDot = tmpDot.getCog();
        double u0 = cogTmpDot.get_u();
        double v0 = cogTmpDot.get_v();
        double half_w = tmpDot.getWidth() / 2.;
        double half_h = tmpDot.getHeight() / 2.;

        if (u >= (u0 - half_w) && u <= (u0 + half_w) && v >= (v0 - half_h) && v <= (v0 + half_h)) {
          // Germ is in a previously detected dot
          good_germ = false;
        }
        ++itnice;
      }

      if (!good_germ)
        continue;

      // Compute the right border position for this possible germ
      unsigned int border_u;
      unsigned int border_v;
      if (findFirstBorder(I, u, v, border_u, border_v) == false) {
        // germ is not good.
        // Jump all the pixels between v,u and v,
        // dotToTest->getFirstBorder_u()
        u = border_u;
        v = border_v;
        continue;
      }

      itbad = badDotsVector.begin();
#define vpBAD_DOT_VALUE (*itbad)
      vpImagePoint cogBadDot;

      while (itbad != badDotsVector.end() && good_germ == true) {
        if ((double)u >= vpBAD_DOT_VALUE.bbox_u_min && (double)u <= vpBAD_DOT_VALUE.bbox_u_max &&
            (double)v >= vpBAD_DOT_VALUE.bbox_v_min && (double)v <= vpBAD_DOT_VALUE.bbox_v_max) {
          std::list<vpImagePoint>::const_iterator it_edges = ip_edges_list.begin();
          while (it_edges != ip_edges_list.end() && good_germ == true) {
            // Test if the germ belong to a previously detected dot:
            // - from the germ go right to the border and compare this
            //   position to the list of pixels of previously detected dots
            cogBadDot = *it_edges;
            // if( border_u == cogBadDot.get_u() && v == cogBadDot.get_v()) {
            if ((std::fabs(border_u - cogBadDot.get_u()) <=
                 vpMath::maximum(std::fabs((double)border_u), std::fabs(cogBadDot.get_u())) *
                     std::numeric_limits<double>::epsilon()) &&
                (std::fabs(v - cogBadDot.get_v()) <=
                 vpMath::maximum(std::fabs((double)v), std::fabs(cogBadDot.get_v())) *
                     std::numeric_limits<double>::epsilon())) {
              good_germ = false;
            }
            ++it_edges;
          }
        }
        ++itbad;
      }
#undef vpBAD_DOT_VALUE

      if (!good_germ) {
        // Jump all the pixels between v,u and v,
        // dotToTest->getFirstBorder_u()
        u = border_u;
        v = border_v;
        continue;
      }

      vpTRACE(4, "Try germ (%d, %d)", u, v);

      vpImagePoint germ;
      germ.set_u(u);
      germ.set_v(v);

      // otherwise estimate the width, height and surface of the dot we
      // created, and test it.
      if (dotToTest != NULL)
        delete dotToTest;
      dotToTest = getInstance();
      dotToTest->setCog(germ);
      dotToTest->setGrayLevelMin(getGrayLevelMin());
      dotToTest->setGrayLevelMax(getGrayLevelMax());
      dotToTest->setGrayLevelPrecision(getGrayLevelPrecision());
      dotToTest->setSizePrecision(getSizePrecision());
      dotToTest->setGraphics(graphics);
      dotToTest->setGraphicsThickness(thickness);
      dotToTest->setComputeMoments(true);
      dotToTest->setArea(area);
      dotToTest->setEllipsoidShapePrecision(ellipsoidShapePrecision);
      dotToTest->setEllipsoidBadPointsPercentage(allowedBadPointsPercentage_);

      // first compute the parameters of the dot.
      // if for some reasons this caused an error tracking
      // (dot partially out of the image...), check the next intersection
      if (dotToTest->computeParameters(I) == false) {
        // Jump all the pixels between v,u and v,
        // dotToTest->getFirstBorder_u()
        u = border_u;
        v = border_v;
        continue;
      }
      // if the dot to test is valid,
      if (dotToTest->isValid(I, *this)) {
        vpImagePoint cogDotToTest = dotToTest->getCog();
        // Compute the distance to the center. The center used here is not the
        // area center available by area.getCenter(area_center_u,
        // area_center_v) but the center of the input area which may be
        // partially outside the image.

        double area_center_u = area_u + area_w / 2.0 - 0.5;
        double area_center_v = area_v + area_h / 2.0 - 0.5;

        double thisDiff_u = cogDotToTest.get_u() - area_center_u;
        double thisDiff_v = cogDotToTest.get_v() - area_center_v;
        double thisDist = sqrt(thisDiff_u * thisDiff_u + thisDiff_v * thisDiff_v);

        bool stopLoop = false;
        itnice = niceDots.begin();

        while (itnice != niceDots.end() && stopLoop == false) {
          tmpDot = *itnice;

          // double epsilon = 0.001; // detecte +sieurs points
          double epsilon = 3.0;
          // if the center of the dot is the same than the current
          // don't add it, test the next point of the grid
          cogTmpDot = tmpDot.getCog();

          if (fabs(cogTmpDot.get_u() - cogDotToTest.get_u()) < epsilon &&
              fabs(cogTmpDot.get_v() - cogDotToTest.get_v()) < epsilon) {
            stopLoop = true;
            // Jump all the pixels between v,u and v,
            // tmpDot->getFirstBorder_u()
            u = border_u;
            v = border_v;
            continue;
          }

          double otherDiff_u = cogTmpDot.get_u() - area_center_u;
          double otherDiff_v = cogTmpDot.get_v() - area_center_v;
          double otherDist = sqrt(otherDiff_u * otherDiff_u + otherDiff_v * otherDiff_v);

          // if the distance of the curent vector element to the center
          // is greater than the distance of this dot to the center,
          // then add this dot before the current vector element.
          if (otherDist > thisDist) {
            niceDots.insert(itnice, *dotToTest);
            ++itnice;
            stopLoop = true;
            // Jump all the pixels between v,u and v,
            // tmpDot->getFirstBorder_u()
            u = border_u;
            v = border_v;
            continue;
          }
          ++itnice;
        }
        vpTRACE(4, "End while (%d, %d)", u, v);

        // if we reached the end of the vector without finding the dot
        // or inserting it, insert it now.
        if (itnice == niceDots.end() && stopLoop == false) {
          niceDots.push_back(*dotToTest);
        }
      } else {
        // Store bad dots
        badDotsVector.push_front(*dotToTest);
      }
    }
  }
  if (dotToTest != NULL)
    delete dotToTest;
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
      (std::fabs(wantedDot.getArea()) > std::numeric_limits<double>::epsilon()))
  // if (size_precision!=0){
  {
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

      if ((wantedDot.getWidth() * size_precision - epsilon < getWidth()) == false) {
        vpDEBUG_TRACE(3, "Bad width > for dot (%g, %g)", cog.get_u(), cog.get_v());
#ifdef DEBUG
        printf("Bad width > for dot (%g, %g)\n", cog.get_u(), cog.get_v());
#endif
        return false;
      }

      if ((getWidth() < wantedDot.getWidth() / (size_precision + epsilon)) == false) {
        vpDEBUG_TRACE(3, "Bad width > for dot (%g, %g)", cog.get_u(), cog.get_v());
#ifdef DEBUG
        printf("Bad width %g > %g for dot (%g, %g)\n", getWidth(), wantedDot.getWidth() / (size_precision + epsilon),
               cog.get_u(), cog.get_v());
#endif
        return false;
      }

      if ((wantedDot.getHeight() * size_precision - epsilon < getHeight()) == false) {
        vpDEBUG_TRACE(3, "Bad height > for dot (%g, %g)", cog.get_u(), cog.get_v());
#ifdef DEBUG
        printf("Bad height %g > %g for dot (%g, %g)\n", wantedDot.getHeight() * size_precision - epsilon, getHeight(),
               cog.get_u(), cog.get_v());
#endif
        return false;
      }

      if ((getHeight() < wantedDot.getHeight() / (size_precision + epsilon)) == false) {
        vpDEBUG_TRACE(3, "Bad height > for dot (%g, %g)", cog.get_u(), cog.get_v());
#ifdef DEBUG
        printf("Bad height %g > %g for dot (%g, %g)\n", getHeight(), wantedDot.getHeight() / (size_precision + epsilon),
               cog.get_u(), cog.get_v());
#endif
        return false;
      }

      if ((wantedDot.getArea() * (size_precision * size_precision) - epsilon < getArea()) == false) {
        vpDEBUG_TRACE(3, "Bad surface > for dot (%g, %g)", cog.get_u(), cog.get_v());
#ifdef DEBUG
        printf("Bad surface %g > %g for dot (%g, %g)\n",
               wantedDot.getArea() * (size_precision * size_precision) - epsilon, getArea(), cog.get_u(), cog.get_v());
#endif
        return false;
      }

      if ((getArea() < wantedDot.getArea() / (size_precision * size_precision + epsilon)) == false) {
        vpDEBUG_TRACE(3, "Bad surface > for dot (%g, %g)", cog.get_u(), cog.get_v());
#ifdef DEBUG
        printf("Bad surface %g < %g for dot (%g, %g)\n", getArea(),
               wantedDot.getArea() / (size_precision * size_precision + epsilon), cog.get_u(), cog.get_v());
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
  int nb_max_bad_points = (int)(nb_point_to_test * allowedBadPointsPercentage_);
  double step_angle = 2 * M_PI / nb_point_to_test;

  //  if (ellipsoidShape_precision != 0 && compute_moment) {
  if (std::fabs(ellipsoidShape_precision) > std::numeric_limits<double>::epsilon() && compute_moment) {
    //       std::cout << "test shape precision......................\n";
    // See F. Chaumette. Image moments: a general and useful set of features
    // for visual servoing. IEEE Trans. on Robotics, 20(4):713-723, August
    // 2004.

    // mu11 = m11 - m00 * xg * yg = m11 - m00 * m10/m00 * m01/m00
    //      = m11 - m10 * m01 / m00
    // mu20 = m20 - m00 * xg^2 = m20 - m00 * m10/m00 * m10/m00
    //      = m20 - m10^2 / m00
    // mu02 = m02 - m01^2 / m00
    // alpha = 1/2 arctan( 2 * mu11 / (mu20 - mu02) )
    //
    // a1^2 = 2 / m00 * (mu02 + mu20 + sqrt( (mu20 - mu02)^2 + 4mu11^2) )
    //
    // a2^2 = 2 / m00 * (mu02 + mu20 - sqrt( (mu20 - mu02)^2 + 4mu11^2) )

    // we compute parameters of the estimated ellipse
    double tmp1 = (m01 * m01 - m10 * m10) / m00 + (m20 - m02);
    double tmp2 = m11 - m10 * m01 / m00;
    double Sqrt = sqrt(tmp1 * tmp1 + 4 * tmp2 * tmp2);
    double a1 = sqrt(2 / m00 * ((m20 + m02) - (m10 * m10 + m01 * m01) / m00 + Sqrt));
    double a2 = sqrt(2 / m00 * ((m20 + m02) - (m10 * m10 + m01 * m01) / m00 - Sqrt));
    double alpha = 0.5 * atan2(2 * (m11 * m00 - m10 * m01), ((m20 - m02) * m00 - m10 * m10 + m01 * m01));

    // to be able to track small dots, minorize the ellipsoid radius for the
    // inner test
    a1 -= 1.0;
    a2 -= 1.0;

    double innerCoef = ellipsoidShape_precision;
    unsigned int u, v;
    double cog_u = this->cog.get_u();
    double cog_v = this->cog.get_v();

    vpImagePoint ip;
    nb_bad_points = 0;
    for (double theta = 0.; theta < 2 * M_PI; theta += step_angle) {
      u = (unsigned int)(cog_u + innerCoef * (a1 * cos(alpha) * cos(theta) - a2 * sin(alpha) * sin(theta)));
      v = (unsigned int)(cog_v + innerCoef * (a1 * sin(alpha) * cos(theta) + a2 * cos(alpha) * sin(theta)));
      if (!this->hasGoodLevel(I, u, v)) {
// 	vpTRACE("Inner circle pixel (%d, %d) has bad level for dot (%g, %g)",
// 		u, v, cog_u, cog_v);
#ifdef DEBUG
        printf("Inner circle pixel (%u, %u) has bad level for dot (%g, %g): "
               "%d not in [%u, %u]\n",
               u, v, cog_u, cog_v, I[v][u], gray_level_min, gray_level_max);
#endif
        // return false;
        nb_bad_points++;
      }
      if (graphics) {
        for (unsigned int t = 0; t < thickness; t++) {
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

    double outCoef = 2 - ellipsoidShape_precision; // 1.6;
    nb_bad_points = 0;
    for (double theta = 0.; theta < 2 * M_PI; theta += step_angle) {
      u = (unsigned int)(cog_u + outCoef * (a1 * cos(alpha) * cos(theta) - a2 * sin(alpha) * sin(theta)));
      v = (unsigned int)(cog_v + outCoef * (a1 * sin(alpha) * cos(theta) + a2 * cos(alpha) * sin(theta)));
#ifdef DEBUG
      // vpDisplay::displayRectangle(I, area, vpColor::yellow);
      vpDisplay::displayCross(I, (int)v, (int)u, 7, vpColor::purple);
      vpDisplay::flush(I);
#endif
      // If outside the area, continue
      if ((double)u < area.getLeft() || (double)u > area.getRight() || (double)v < area.getTop() ||
          (double)v > area.getBottom()) {
        continue;
      }
      if (!this->hasReverseLevel(I, u, v)) {
// 	vpTRACE("Outside circle pixel (%d, %d) has bad level for dot (%g,
// %g)", 		u, v, cog_u, cog_v);
#ifdef DEBUG
        printf("Outside circle pixel (%u, %u) has bad level for dot (%g, "
               "%g): %d not in [%u, %u]\n",
               u, v, cog_u, cog_v, I[v][u], gray_level_min, gray_level_max);
#endif
        nb_bad_points++;
        // return false;
      }
      if (graphics) {
        for (unsigned int t = 0; t < thickness; t++) {
          ip.set_u(u + t);
          ip.set_v(v);

          vpDisplay::displayPoint(I, ip, vpColor::green);
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
  if (!isInArea(u, v))
    return false;

  if (I[v][u] >= gray_level_min && I[v][u] <= gray_level_max) {
    return true;
  } else {
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

  if (!isInArea(u, v))
    return false;

  if (I[v][u] < gray_level_min || I[v][u] > gray_level_max) {
    return true;
  } else {
    return false;
  }
}

/*!
  Return a new instance of vpDot2.

  Should be used by child classed to return their own instance of vpDot2.

  \return An instance of vpDot2.

*/
vpDot2 *vpDot2::getInstance() { return new vpDot2(); }

/*!

  Returns the list of Freeman chain code used to turn around the dot
  counterclockwise.

  \return List of Freeman chain list [0, ..., 7]
  - 0 : right
  - 1 : top right
  - 2 : top
  - 3 : top left
  - 4 : left
  - 5 : down left
  - 6 : down
  - 7 : down right
*/
void vpDot2::getFreemanChain(std::list<unsigned int> &freeman_chain) const { freeman_chain = direction_list; }

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
bool vpDot2::computeParameters(const vpImage<unsigned char> &I, const double &_u, const double &_v)
{
  direction_list.clear();
  ip_edges_list.clear();

  double est_u = _u; // estimated
  double est_v = _v;

  // if u has default value, set it to the actual center value
  // if( est_u == -1.0 )
  if (std::fabs(est_u + 1.0) <= vpMath::maximum(std::fabs(est_u), 1.) * std::numeric_limits<double>::epsilon()) {
    est_u = this->cog.get_u();
  }

  // if v has default value, set it to the actual center value
  // if( est_v == -1.0 )
  if (std::fabs(est_v + 1.0) <= vpMath::maximum(std::fabs(est_v), 1.) * std::numeric_limits<double>::epsilon()) {
    est_v = this->cog.get_v();
  }

  // if the estimated position of the dot is out of the image, not need to
  // continue, return an error tracking
  if (!isInArea((unsigned int)est_u, (unsigned int)est_v)) {
    vpDEBUG_TRACE(3,
                  "Initial pixel coordinates (%d, %d) for dot tracking are "
                  "not in the area",
                  (int)est_u, (int)est_v);
    return false;
  }

  bbox_u_min = (int)I.getWidth();
  bbox_u_max = 0;
  bbox_v_min = (int)I.getHeight();
  bbox_v_max = 0;

  // if the first point doesn't have the right level then there's no point to
  // continue.
  if (!hasGoodLevel(I, (unsigned int)est_u, (unsigned int)est_v)) {
    vpDEBUG_TRACE(3, "Can't find a dot from pixel (%d, %d) coordinates", (int)est_u, (int)est_v);
    return false;
  }

  // find the border

  if (!findFirstBorder(I, (unsigned int)est_u, (unsigned int)est_v, this->firstBorder_u, this->firstBorder_v)) {

    vpDEBUG_TRACE(3, "Can't find first border (%d, %d) coordinates", (int)est_u, (int)est_v);
    return false;
  }

  unsigned int dir = 6;

  // Determine the first element of the Freeman chain
  computeFreemanChainElement(I, this->firstBorder_u, this->firstBorder_v, dir);
  unsigned int firstDir = dir;

  // if we are now out of the image, return an error tracking
  if (!isInArea(this->firstBorder_u, this->firstBorder_v)) {
    vpDEBUG_TRACE(3, "Border pixel coordinates (%d, %d) of the dot are not in the area", this->firstBorder_u,
                  this->firstBorder_v);
    return false;
  }

  // store the new direction and dot border coordinates.
  direction_list.push_back(dir);
  vpImagePoint ip;
  ip.set_u(this->firstBorder_u);
  ip.set_v(this->firstBorder_v);

  ip_edges_list.push_back(ip);

  int border_u = (int)this->firstBorder_u;
  int border_v = (int)this->firstBorder_v;

  //   vpTRACE("-----------------------------------------");
  //   vpTRACE("first border_u: %d border_v: %d dir: %d",
  // 	this->firstBorder_u, this->firstBorder_v,firstDir);
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
    if (graphics) {
      for (int t = 0; t < (int)thickness; t++) {
        ip.set_u(border_u + t);
        ip.set_v(border_v);

        vpDisplay::displayPoint(I, ip, vpColor::red);
      }
      // vpDisplay::flush(I);
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
    if (compute_moment) {
      m11 += dMuv; // Second order moment
      m20 += dMu2; // Second order moment along v axis
      m02 += dMv2; // Second order moment along u axis
    }
    // if we are now out of the image, return an error tracking
    if (!isInArea((unsigned int)border_u, (unsigned int)border_v)) {

      vpDEBUG_TRACE(3, "Dot (%d, %d) is not in the area", border_u, border_v);
      // Can Occur on a single pixel dot located on the top border
      return false;
    }

    // store the new direction and dot border coordinates.

    direction_list.push_back(dir);

    ip.set_u(border_u);
    ip.set_v(border_v);
    ip_edges_list.push_back(ip);

    // vpDisplay::getClick(I);

    // update the extreme point of the dot.
    if (border_v < bbox_v_min)
      bbox_v_min = border_v;
    if (border_v > bbox_v_max)
      bbox_v_max = border_v;
    if (border_u < bbox_u_min)
      bbox_u_min = border_u;
    if (border_u > bbox_u_max)
      bbox_u_max = border_u;

    // move around the tracked entity by following the border.
    if (computeFreemanChainElement(I, (unsigned int)border_u, (unsigned int)border_v, dir) == false) {
      vpDEBUG_TRACE(3, "Can't compute Freeman chain for dot (%d, %d)", border_u, border_v);
      return false;
    }

    //     vpTRACE("border_u: %d border_v: %d dir: %d", border_u, border_v,
    //     dir);

  } while ((getFirstBorder_u() != (unsigned int)border_u || getFirstBorder_v() != (unsigned int)border_v ||
            firstDir != dir) &&
           isInArea((unsigned int)border_u, (unsigned int)border_v));

#ifdef VP_DEBUG
#if VP_DEBUG_MODE == 3
  vpDisplay::flush(I);
#endif
#endif

  // if the surface is one or zero , the center of gravity wasn't properly
  // detected. Return an error tracking.
  // if( m00 == 0 || m00 == 1 )
  if (std::fabs(m00) <= std::numeric_limits<double>::epsilon() ||
      std::fabs(m00 - 1.) <= vpMath::maximum(std::fabs(m00), 1.) * std::numeric_limits<double>::epsilon()) {
    vpDEBUG_TRACE(3, "The center of gravity of the dot wasn't properly detected");
    return false;
  } else // compute the center
  {
    // this magic formula gives the coordinates of the center of gravity
    double tmpCenter_u = m10 / m00;
    double tmpCenter_v = m01 / m00;

    // Updates the central moments
    if (compute_moment) {
      mu11 = m11 - tmpCenter_u * m01;
      mu02 = m02 - tmpCenter_v * m01;
      mu20 = m20 - tmpCenter_u * m10;
    }

    // check the center is in the image... never know...
    //     if( !hasGoodLevel( I, (unsigned int)tmpCenter_u,
    // 		       (unsigned int)tmpCenter_v ) )
    //     {
    //       vpDEBUG_TRACE(3, "The center of gravity of the dot (%g, %g) has
    //       not a good in level", tmpCenter_u, tmpCenter_v); return false;
    //     }

    cog.set_u(tmpCenter_u);
    cog.set_v(tmpCenter_v);
  }

  width = bbox_u_max - bbox_u_min + 1;
  height = bbox_v_max - bbox_v_min + 1;
  surface = m00;

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
  // rounding problems and it's actually more a trouble than smth else to
  // work with double when navigating around the dot.
  border_u = u;
  border_v = v;
  double epsilon = 0.001;

#ifdef DEBUG
  std::cout << "gray level: " << gray_level_min << " " << gray_level_max << std::endl;
#endif
  while (hasGoodLevel(I, border_u + 1, border_v) && border_u < area.getRight() /*I.getWidth()*/) {
    // if the width of this dot was initialised and we already crossed the dot
    // on more than the max possible width, no need to continue, return an
    // error tracking
    if (getWidth() > 0 && (border_u - u) > getWidth() / (getMaxSizeSearchDistancePrecision() + epsilon)) {
      vpDEBUG_TRACE(3,
                    "The found dot (%d, %d, %d) has a greater width than the "
                    "required one",
                    u, v, border_u);
      return false;
    }
#ifdef DEBUG
    vpDisplay::displayPoint(I, (int)border_v, (int)border_u + 1, vpColor::green);
    vpDisplay::flush(I);
#endif

    border_u++;
  }
  return true;
}

/*!

  Considering a pixel (u, v) compute the next element of the Freeman chain
  code.

  According to the gray level of pixel (u, v) and his eight neighbors
  determine the next element of the chain in order to turn around the dot
  counterclockwise.

  \param I : The image we are working with.
  \param v : The row coordinate of a pixel on a border.
  \param u : The column coordinate of the pixel on a border.
  \param element : The next freeman element chain code (0, 1, 2, 3, 4, 5, 6,
  7) with 0 for right moving, 2 for down, 4 for left and 6 for up moving.

  \return false if an element cannot be found. Occurs for example with an area
  constituted by a single pixel. Return true if success.
*/
bool vpDot2::computeFreemanChainElement(const vpImage<unsigned char> &I, const unsigned int &u, const unsigned int &v,
                                        unsigned int &element)
{

  if (hasGoodLevel(I, u, v)) {
    unsigned int _u = u;
    unsigned int _v = v;
    // get the point on the right of the point passed in
    updateFreemanPosition(_u, _v, (element + 2) % 8);
    if (hasGoodLevel(I, _u, _v)) {
      element = (element + 2) % 8; // turn right
    } else {
      unsigned int _u1 = u;
      unsigned int _v1 = v;
      updateFreemanPosition(_u1, _v1, (element + 1) % 8);

      if (hasGoodLevel(I, _u1, _v1)) {
        element = (element + 1) % 8; // turn diag right
      } else {
        unsigned int _u2 = u;
        unsigned int _v2 = v;
        updateFreemanPosition(_u2, _v2, element); // same direction

        if (hasGoodLevel(I, _u2, _v2)) {
          // element = element;      // keep same dir
        } else {
          unsigned int _u3 = u;
          unsigned int _v3 = v;
          updateFreemanPosition(_u3, _v3, (element + 7) % 8); // diag left

          if (hasGoodLevel(I, _u3, _v3)) {
            element = (element + 7) % 8; // turn diag left
          } else {
            unsigned int _u4 = u;
            unsigned int _v4 = v;
            updateFreemanPosition(_u4, _v4, (element + 6) % 8); // left

            if (hasGoodLevel(I, _u4, _v4)) {
              element = (element + 6) % 8; // turn left
            } else {
              unsigned int _u5 = u;
              unsigned int _v5 = v;
              updateFreemanPosition(_u5, _v5, (element + 5) % 8); // left

              if (hasGoodLevel(I, _u5, _v5)) {
                element = (element + 5) % 8; // turn diag down
              } else {
                unsigned int _u6 = u;
                unsigned int _v6 = v;
                updateFreemanPosition(_u6, _v6, (element + 4) % 8); // left

                if (hasGoodLevel(I, _u6, _v6)) {
                  element = (element + 4) % 8; // turn down
                } else {
                  unsigned int _u7 = u;
                  unsigned int _v7 = v;
                  updateFreemanPosition(_u7, _v7, (element + 3) % 8); // diag

                  if (hasGoodLevel(I, _u7, _v7)) {
                    element = (element + 3) % 8; // turn diag right down
                  } else {
                    // No neighbor with a good level
                    //
                    return false;
                  }
                }
              }
            }
          }
        }
      }
    }
  }

  else {
    return false;
  }

  return true;
}

/*!

  Given the previous position of a pixel (u_p, v_p) on the dot border and the
  direction to reach the next pixel on the border, compute Freeman parameters.

  \param u_p : Previous value of the row coordinate of a pixel on a border.
  \param v_p : Previous value of the column coordinate of a pixel on a border.
  \param du : Increment to go from previous to next pixel on the dot border.
  \param dv : Increment to go from previous to next pixel on the dot border.

  \param dS : Enclosed area increases. Cumulated values of dS gives m00.

  \param dMu : First order moment along v axis increases. Cumulated values of
  dMu gives m10.

  \param dMv : First order moment along u axis increases. Cumulated values of
  dMv gives m01.

  \param dMuv : Moment increases. Cumulated values of dMuv gives m11.

  \param dMu2 : Second order moment along v axis increases. Cumulated values
  of dMu2 gives m20.

  \param dMv2 : Second order moment along u axis increases. Cumulated values
  of dMv2 gives m02.

  Considering the previous coordinates (u_p, v_p) of a pixel on a border, the
  next coordinates (u, v) are given by: u = u_p + du and v = v_p + dv


*/
void vpDot2::computeFreemanParameters(const int &u_p, const int &v_p, unsigned int &element, int &du, int &dv,
                                      float &dS, float &dMu, float &dMv, float &dMuv, float &dMu2, float &dMv2)
{
  du = 0;
  dv = 0;
  dMuv = 0;
  dMu2 = 0;
  dMv2 = 0;

  /*
           3  2  1
            \ | /
             \|/
         4 ------- 0
             /|\
            / | \
           5  6  7
  */
  switch (element) {
  case 0: // go right
    du = 1;
    dS = (float)v_p;
    dMu = 0.0;
    dMv = (float)(0.5 * v_p * v_p);
    if (compute_moment) {
      dMuv = (float)(0.25 * v_p * v_p * (2 * u_p + 1));
      dMu2 = 0;
      dMv2 = (float)(1.0 / 3. * v_p * v_p * v_p);
    }
    break;

  case 1: // go right top
    du = 1;
    dv = 1;
    dS = (float)(v_p + 0.5);
    dMu = -(float)(0.5 * u_p * (u_p + 1) + 1.0 / 6.0);
    dMv = (float)(0.5 * v_p * (v_p + 1) + 1.0 / 6.0);
    if (compute_moment) {
      float half_u_p = (float)(0.5 * u_p);
      dMuv = (float)(v_p * v_p * (0.25 + half_u_p) + v_p * (1. / 3. + half_u_p) + 1. / 6. * u_p + 0.125);
      dMu2 = (float)(-1. / 3. * u_p * (u_p * u_p + 1.5 * u_p + 1.) - 1. / 12.0);
      dMv2 = (float)(1. / 3. * v_p * (v_p * v_p + 1.5 * v_p + 1.) + 1. / 12.0);
    }
    break;

  case 2: // go top
    dv = 1;
    dS = 0.0;
    dMu = (float)(-0.5 * u_p * u_p);
    dMv = 0.0;
    if (compute_moment) {
      dMuv = 0;
      dMu2 = (float)(-1.0 / 3. * u_p * u_p * u_p);
      dMv2 = 0;
    }
    break;

  case 3:
    du = -1;
    dv = 1;
    dS = (float)(-v_p - 0.5);
    dMu = -(float)(0.5 * u_p * (u_p - 1) + 1.0 / 6.0);
    dMv = -(float)(0.5 * v_p * (v_p + 1) + 1.0 / 6.0);
    if (compute_moment) {
      float half_u_p = (float)(0.5 * u_p);
      dMuv = (float)(v_p * v_p * (0.25 - half_u_p) + v_p * (1. / 3. - half_u_p) - 1. / 6. * u_p + 0.125);
      dMu2 = (float)(-1. / 3. * u_p * (u_p * u_p - 1.5 * u_p + 1.) - 1. / 12.0);
      dMv2 = (float)(-1. / 3. * v_p * (v_p * v_p + 1.5 * v_p + 1.) - 1. / 12.0);
    }
    break;

  case 4:
    du = -1;
    dS = (float)(-v_p);
    dMv = (float)(-0.5 * v_p * v_p);
    dMu = 0.0;
    if (compute_moment) {
      dMuv = (float)(-0.25 * v_p * v_p * (2 * u_p - 1));
      dMu2 = 0;
      dMv2 = (float)(-1.0 / 3. * v_p * v_p * v_p);
    }
    break;

  case 5:
    du = -1;
    dv = -1;
    dS = (float)(-v_p + 0.5);
    dMu = (float)(0.5 * u_p * (u_p - 1) + 1.0 / 6.0);
    dMv = (float)(-(0.5 * v_p * (v_p - 1) + 1.0 / 6.0));
    if (compute_moment) {
      float half_u_p = (float)(0.5 * u_p);
      dMuv = (float)(v_p * v_p * (0.25 - half_u_p) - v_p * (1. / 3. - half_u_p) - 1. / 6. * u_p + 0.125);
      dMu2 = (float)(1. / 3. * u_p * (u_p * u_p - 1.5 * u_p + 1.) - 1. / 12.0);
      dMv2 = (float)(-1. / 3. * v_p * (v_p * v_p - 1.5 * v_p + 1.) - 1. / 12.0);
    }
    break;

  case 6:
    dv = -1;
    dS = 0.0;
    dMu = (float)(0.5 * u_p * u_p);
    dMv = 0.0;
    if (compute_moment) {
      dMuv = 0;
      dMu2 = (float)(1.0 / 3. * u_p * u_p * u_p);
      dMv2 = 0;
    }
    break;

  case 7:
    du = 1;
    dv = -1;
    dS = (float)(v_p - 0.5);
    dMu = (float)(0.5 * u_p * (u_p + 1) + 1.0 / 6.0);
    dMv = (float)(0.5 * v_p * (v_p - 1) + 1.0 / 6.0);
    if (compute_moment) {
      float half_u_p = (float)(0.5 * u_p);
      dMuv = (float)(v_p * v_p * (0.25 + half_u_p) - v_p * (1. / 3. + half_u_p) + 1. / 6. * u_p + 0.125);
      dMu2 = (float)(1. / 3. * u_p * (u_p * u_p + 1.5 * u_p + 1.) + 1. / 12.0);
      dMv2 = (float)(1. / 3. * v_p * (v_p * v_p - 1.5 * v_p + 1.) - 1. / 12.0);
    }
    break;
  }
}

/*!

  From a pixel coordinate and a direction, update the pixel coordinates after
  moving forward.

  \param v : The row coordinate of the pixel, updated by this method.

  \param u : The column coordinate of the pixel, updated by this method.

  \param dir : The direction in the image, 0=right, 1, 2=down, 3, 4=left, 5,
  6=up and 7.

*/
void vpDot2::updateFreemanPosition(unsigned int &u, unsigned int &v, const unsigned int &dir)
{
  switch (dir) {
  case 0:
    u += 1;
    break;
  case 1:
    u += 1;
    v += 1;
    break;
  case 2:
    v += 1;
    break;
  case 3:
    u -= 1;
    v += 1;
    break;
  case 4:
    u -= 1;
    break;
  case 5:
    u -= 1;
    v -= 1;
    break;
  case 6:
    v -= 1;
    break;
  case 7:
    u += 1;
    v -= 1;
    break;
  }
}

/*!

  Test if a pixel is in the image. Points of the border are not
  considered to be in the image.  Call the isInImage( vpImage<unsigned
  char> &I, const vpImagePoint &) method.

  \param I : The image.

  \return true if the pixel of coordinates (posI, posJ) is in the image and
  false otherwise.
*/
bool vpDot2::isInImage(const vpImage<unsigned char> &I) const { return isInImage(I, cog); }

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

  if (u < 0 || u >= w)
    return false;
  if (v < 0 || v >= h)
    return false;
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
  unsigned int area_u_min = (unsigned int)area.getLeft();
  unsigned int area_u_max = (unsigned int)area.getRight();
  unsigned int area_v_min = (unsigned int)area.getTop();
  unsigned int area_v_max = (unsigned int)area.getBottom();

  if (u < area_u_min || u > area_u_max)
    return false;
  if (v < area_v_min || v > area_v_max)
    return false;
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
  gridWidth = (unsigned int)(getWidth() * getMaxSizeSearchDistancePrecision() / sqrt(2.));
  gridHeight = (unsigned int)(getHeight() * getMaxSizeSearchDistancePrecision() / sqrt(2.0));

  if (gridWidth == 0)
    gridWidth = 1;
  if (gridHeight == 0)
    gridHeight = 1;
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
  int cog_u = (int)cog.get_u();
  int cog_v = (int)cog.get_v();

  unsigned int sum_value = 0;
  unsigned int nb_pixels = 0;

  for (unsigned int i = (unsigned int)this->bbox_u_min; i <= (unsigned int)this->bbox_u_max; i++) {
    unsigned int pixel_gray = (unsigned int)I[(unsigned int)cog_v][i];
    if (pixel_gray >= getGrayLevelMin() && pixel_gray <= getGrayLevelMax()) {
      sum_value += pixel_gray;
      nb_pixels++;
    }
  }
  for (unsigned int i = (unsigned int)this->bbox_v_min; i <= (unsigned int)this->bbox_v_max; i++) {
    unsigned char pixel_gray = I[i][(unsigned int)cog_u];
    if (pixel_gray >= getGrayLevelMin() && pixel_gray <= getGrayLevelMax()) {
      sum_value += pixel_gray;
      nb_pixels++;
    }
  }
  if (nb_pixels < 10) { // could be good to choose the min nb points from area of dot
    // add diagonals points to have enough point
    int imin, imax;
    if ((cog_u - bbox_u_min) > (cog_v - bbox_v_min)) {
      imin = cog_v - bbox_v_min;
    } else {
      imin = cog_u - bbox_u_min;
    }
    if ((bbox_u_max - cog_u) > (bbox_v_max - cog_v)) {
      imax = bbox_v_max - cog_v;
    } else {
      imax = bbox_u_max - cog_u;
    }
    for (int i = -imin; i <= imax; i++) {
      unsigned int pixel_gray = (unsigned int)I[(unsigned int)(cog_v + i)][(unsigned int)(cog_u + i)];
      if (pixel_gray >= getGrayLevelMin() && pixel_gray <= getGrayLevelMax()) {
        sum_value += pixel_gray;
        nb_pixels++;
      }
    }

    if ((cog_u - bbox_u_min) > (bbox_v_max - cog_v)) {
      imin = bbox_v_max - cog_v;
    } else {
      imin = cog_u - bbox_u_min;
    }
    if ((bbox_u_max - cog_u) > (cog_v - bbox_v_min)) {
      imax = cog_v - bbox_v_min;
    } else {
      imax = bbox_u_max - cog_u;
    }

    for (int i = -imin; i <= imax; i++) {
      unsigned char pixel_gray = I[(unsigned int)(cog_v - i)][(unsigned int)(cog_u + i)];
      if (pixel_gray >= getGrayLevelMin() && pixel_gray <= getGrayLevelMax()) {
        sum_value += pixel_gray;
        nb_pixels++;
      }
    }
  }

  if (nb_pixels == 0) {
    // should never happen
    throw(vpTrackingException(vpTrackingException::notEnoughPointError, "No point was found"));
  } else {
    mean_gray_level = sum_value / nb_pixels;
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
      for (i = 0; i < n; ++i) {
        cog.set_uv(Cogs[i][0], Cogs[i][1]);
        dot[i].setGraphics(true);
        dot[i].setCog(cog);
        if (trackDot) {
          dot[i].initTracking(I, cog);
          dot[i].track(I);
          vpDisplay::displayCross(I, cog, 10, col);
        }
      }
    } catch (...) {
      std::cout << "Cannot track dots from file" << std::endl;
      fromFile = false;
    }
    vpDisplay::flush(I);

    // check that dots are far away ones from the other
    for (i = 0; i < n && fromFile; ++i) {
      double d = sqrt(vpMath::sqr(dot[i].getHeight()) + vpMath::sqr(dot[i].getWidth()));
      for (unsigned int j = 0; j < n && fromFile; ++j)
        if (j != i)
          if (dot[i].getDistance(dot[j]) < d) {
            fromFile = false;
            std::cout << "Dots from file seem incoherent" << std::endl;
          }
    }
  }

  if (!fromFile) {
    vpDisplay::display(I);
    vpDisplay::flush(I);

    std::cout << "Click on the " << n << " dots clockwise starting from upper/left dot..." << std::endl;
    for (i = 0; i < n; i++) {
      if (trackDot) {
        dot[i].setGraphics(true);
        dot[i].initTracking(I);
        cog = dot[i].getCog();
      } else {
        vpDisplay::getClick(I, cog);
        dot[i].setCog(cog);
      }
      Cogs[i][0] = cog.get_u();
      Cogs[i][1] = cog.get_v();
      vpDisplay::displayCross(I, cog, 10, col);
      vpDisplay::flush(I);
    }
  }

  if (!fromFile && (dotFile != "")) {
    vpMatrix::saveMatrix(dotFile, Cogs);
    std::cout << Cogs.getRows() << " dots written to file " << dotFile << std::endl;
  }

  // back to non graphic mode
  for (i = 0; i < n; ++i)
    dot[i].setGraphics(false);

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
  vpImagePoint indicating the desired position (default NULL), will be
  displayed in red
*/
void vpDot2::trackAndDisplay(vpDot2 dot[], const unsigned int &n, vpImage<unsigned char> &I,
                             std::vector<vpImagePoint> &cogs, vpImagePoint *cogStar)
{
  unsigned int i;
  // tracking
  for (i = 0; i < n; ++i) {
    dot[i].track(I);
    cogs.push_back(dot[i].getCog());
  }
  // trajectories
  for (i = n; i < cogs.size(); ++i)
    vpDisplay::displayCircle(I, cogs[i], 4, vpColor::green, true);
  // initial position
  for (i = 0; i < n; ++i)
    vpDisplay::displayCircle(I, cogs[i], 4, vpColor::blue, true);
  // if exists, desired position
  if (cogStar != NULL)
    for (i = 0; i < n; ++i) {
      vpDisplay::displayDotLine(I, cogStar[i], dot[i].getCog(), vpColor::red);
      vpDisplay::displayCircle(I, cogStar[i], 4, vpColor::red, true);
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
  vpDisplay::displayCross(I, cog, 3 * thickness + 8, color, thickness);
  std::list<vpImagePoint>::const_iterator it;

  for (it = edges_list.begin(); it != edges_list.end(); ++it) {
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
  vpDisplay::displayCross(I, cog, 3 * thickness + 8, color, thickness);
  std::list<vpImagePoint>::const_iterator it;

  for (it = edges_list.begin(); it != edges_list.end(); ++it) {
    vpDisplay::displayPoint(I, *it, color);
  }
}

/*!
  Writes the dot center of gravity coordinates in the frame (i,j) (For more
  details about the orientation of the frame see the vpImagePoint
  documentation) to the stream \e os, and returns a reference to the stream.
*/
VISP_EXPORT std::ostream &operator<<(std::ostream &os, vpDot2 &d) { return (os << "(" << d.getCog() << ")"); }
