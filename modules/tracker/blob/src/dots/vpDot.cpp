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
 * Eric Marchand
 * Fabien Spindler
 * Aurelien Yol
 *
 *****************************************************************************/

/*
  \file vpDot.cpp
  \brief Track a white dot
*/

#include <visp3/blob/vpDot.h>
#include <visp3/core/vpColor.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpTrackingException.h>

#include <vector>

/*
  \class vpDot
  \brief Track a white dot
*/

/* spiral size for the dot search */
const unsigned int vpDot::SPIRAL_SEARCH_SIZE = 350;

/*!

  Initialize the tracker with default parameters.
  - connexityType is set to 4 (see setConnexityType())
  - dot maximal size is set to 25% of the image size (see setMaxDotSize())
*/
void vpDot::init()
{
  cog.set_u(0);
  cog.set_v(0);

  compute_moment = false;
  graphics = false;
  thickness = 1;
  maxDotSizePercentage = 0.25; // 25 % of the image size

  mean_gray_level = 0;
  gray_level_min = 128;
  gray_level_max = 255;
  grayLevelPrecision = 0.85;
  gamma = 1.5;

  m00 = m11 = m02 = m20 = m10 = m01 = mu11 = mu02 = mu20 = 0;

  connexityType = CONNEXITY_4;

  u_min = u_max = v_min = v_max = 0;

  gray_level_out = 0;
  nbMaxPoint = 0;
}

vpDot::vpDot()
  : m00(0.), m01(0.), m10(0.), m11(0.), m20(0.), m02(0.), mu11(0.), mu20(0.), mu02(0.), ip_connexities_list(),
    ip_edges_list(), connexityType(CONNEXITY_4), cog(), u_min(0), u_max(0), v_min(0), v_max(0), graphics(false),
    thickness(1), maxDotSizePercentage(0.25), gray_level_out(0), mean_gray_level(0), gray_level_min(128),
    gray_level_max(255), grayLevelPrecision(0.85), gamma(1.5), compute_moment(false), nbMaxPoint(0)
{
}

/*!
  \brief Constructor with initialization of the dot location.

  \param ip : An image point with sub-pixel coordinates.
 */
vpDot::vpDot(const vpImagePoint &ip)
  : m00(0.), m01(0.), m10(0.), m11(0.), m20(0.), m02(0.), mu11(0.), mu20(0.), mu02(0.), ip_connexities_list(),
    ip_edges_list(), connexityType(CONNEXITY_4), cog(ip), u_min(0), u_max(0), v_min(0), v_max(0), graphics(false),
    thickness(1), maxDotSizePercentage(0.25), gray_level_out(0), mean_gray_level(0), gray_level_min(128),
    gray_level_max(255), grayLevelPrecision(0.85), gamma(1.5), compute_moment(false), nbMaxPoint(0)
{
}

/*!
  \brief Copy constructor.
 */
vpDot::vpDot(const vpDot &d)
  : vpTracker(d), m00(0.), m01(0.), m10(0.), m11(0.), m20(0.), m02(0.), mu11(0.), mu20(0.), mu02(0.),
    ip_connexities_list(), ip_edges_list(), connexityType(CONNEXITY_4), cog(), u_min(0), u_max(0), v_min(0), v_max(0),
    graphics(false), thickness(1), maxDotSizePercentage(0.25), gray_level_out(0), mean_gray_level(0),
    gray_level_min(128), gray_level_max(255), grayLevelPrecision(0.85), gamma(1.5), compute_moment(false), nbMaxPoint(0)
{
  *this = d;
}

/*!
  \brief Destructor.
 */
vpDot::~vpDot() { ip_connexities_list.clear(); }

/*!
  \brief Copy operator.
 */
vpDot &vpDot::operator=(const vpDot &d)
{
  ip_edges_list = d.ip_edges_list;
  ip_connexities_list = d.ip_connexities_list;
  connexityType = d.connexityType;
  cog = d.getCog();

  u_min = d.u_min;
  v_min = d.v_min;
  u_max = d.u_max;
  v_max = d.v_max;

  graphics = d.graphics;
  thickness = d.thickness;
  maxDotSizePercentage = d.maxDotSizePercentage;
  gray_level_out = d.gray_level_out;
  mean_gray_level = d.mean_gray_level;
  gray_level_min = d.gray_level_min;
  gray_level_max = d.gray_level_max;
  grayLevelPrecision = d.grayLevelPrecision;
  gamma = d.gamma;
  compute_moment = d.compute_moment;
  nbMaxPoint = d.nbMaxPoint;

  m00 = d.m00;
  m01 = d.m01;
  m10 = d.m10;
  m11 = d.m11;
  m02 = d.m02;
  m20 = d.m20;

  mu11 = d.mu11;
  mu20 = d.mu20;
  mu02 = d.mu02;

  return *this;
}

bool vpDot::operator!=(const vpDot &d) const { return (cog != d.getCog()); }

bool vpDot::operator==(const vpDot &d) const { return (cog == d.getCog()); }

/*!

  This method choose a gray level (default is 0) used to modify the
  "in" dot level in "out" dot level. This gray level is here needed to
  stop the recursivity . The pixels of the dot are set to this new
  gray level "\out\" when connexe() is called.

  \exception vpTrackingException::initializationError : No valid gray
  level "out" can be found. This means that the min gray level is set to 0
  and the max gray level to 255. This should not occur
*/
void vpDot::setGrayLevelOut()
{
  if (gray_level_min == 0) {
    if (gray_level_max == 255) {
      // gray_level_min = 0 and gray_level_max = 255: this should not occur
      // vpERROR_TRACE("Unable to choose a good \"out\" level") ;
      throw(vpTrackingException(vpTrackingException::initializationError, "Unable to choose a good \"out\" level"));
    }
    gray_level_out = static_cast<unsigned char>(gray_level_max + 1u);
  }
}

/*!
  Perform the tracking of a dot by connex components.

  \param mean_value : Threshold to use for the next call to track()
  and corresponding to the mean value of the dot intensity.

  \warning The content of the image is modified thanks to
  setGrayLevelOut() called before. This method choose a gray level
  (default is 0) used to modify the "in" dot level in "out" dot
  level. This gray level is here needed to stop the recursivity . The
  pixels of the dot are set to this new gray level "\out\".

  \return vpDot::out if an error occurs, vpDot::in otherwise.

  \sa setGrayLevelOut()
*/

bool vpDot::connexe(const vpImage<unsigned char> &I, unsigned int u, unsigned int v, double &mean_value, double &u_cog,
                    double &v_cog, double &n)
{
  std::vector<bool> checkTab(I.getWidth() * I.getHeight(), false);
  return connexe(I, u, v, mean_value, u_cog, v_cog, n, checkTab);
}
/*!
  Perform the tracking of a dot by connex components.

  \param mean_value : Threshold to use for the next call to track()
  and corresponding to the mean value of the dot intensity.

  \warning The content of the image is modified thanks to
  setGrayLevelOut() called before. This method choose a gray level
  (default is 0) used to modify the "in" dot level in "out" dot
  level. This gray level is here needed to stop the recursivity . The
  pixels of the dot are set to this new gray level "\out\".

  \return vpDot::out if an error occurs, vpDot::in otherwise.

  \sa setGrayLevelOut()
*/

bool vpDot::connexe(const vpImage<unsigned char> &I, unsigned int u, unsigned int v, double &mean_value, double &u_cog,
                    double &v_cog, double &n, std::vector<bool> &checkTab)
{

  unsigned int width = I.getWidth();
  unsigned int height = I.getHeight();

  // Test if we are in the image
  if ((u >= width) || (v >= height)) {
    // std::cout << "out of bound" << std::endl;
    return false;
  }

  if (checkTab[u + v * I.getWidth()])
    return true;

  vpImagePoint ip;
  ip.set_u(u);
  ip.set_v(v);

  if (I[v][u] >= gray_level_min && I[v][u] <= gray_level_max) {
    checkTab[v * I.getWidth() + u] = true;

    ip_connexities_list.push_back(ip);

    u_cog += u;
    v_cog += v;
    n += 1;

    if (n > nbMaxPoint) {
      //      vpERROR_TRACE("Too many point %lf (%lf%% of image size). "
      //		    "This threshold can be modified using the
      // setMaxDotSize() " 		    "method.", 		    n, n /
      //(I.getWidth() * I.getHeight()), 		    nbMaxPoint,
      // maxDotSizePercentage) ;

      throw(vpTrackingException(vpTrackingException::featureLostError,
                                "Too many point %lf (%lf%% of image size). "
                                "This threshold can be modified using the setMaxDotSize() "
                                "method.",
                                n, n / (I.getWidth() * I.getHeight()), nbMaxPoint, maxDotSizePercentage));
    }

    // Bounding box update
    if (u < this->u_min)
      this->u_min = u;
    if (u > this->u_max)
      this->u_max = u;
    if (v < this->v_min)
      this->v_min = v;
    if (v > this->v_max)
      this->v_max = v;

    // Mean value of the dot intensities
    mean_value = (mean_value * (n - 1) + I[v][u]) / n;
    if (compute_moment == true) {
      m00++;
      m10 += u;
      m01 += v;
      m11 += (u * v);
      m20 += u * u;
      m02 += v * v;
    }
  } else {
    // std::cout << "not in" << std::endl;
    return false;
  }

  bool edge = false;

  // if((int)u-1 >= 0)
  if (u >= 1)
    if (!checkTab[u - 1 + v * I.getWidth()])
      if (!connexe(I, u - 1, v, mean_value, u_cog, v_cog, n, checkTab))
        edge = true;

  if (u + 1 < I.getWidth())
    if (!checkTab[u + 1 + v * I.getWidth()])
      if (!connexe(I, u + 1, v, mean_value, u_cog, v_cog, n, checkTab))
        edge = true;

  if (v >= 1)
    if (!checkTab[u + (v - 1) * I.getWidth()])
      if (!connexe(I, u, v - 1, mean_value, u_cog, v_cog, n, checkTab))
        edge = true;

  if (v + 1 < I.getHeight())
    if (!checkTab[u + (v + 1) * I.getWidth()])
      if (!connexe(I, u, v + 1, mean_value, u_cog, v_cog, n, checkTab))
        edge = true;

  if (connexityType == CONNEXITY_8) {
    if (v >= 1 && u >= 1)
      if (!checkTab[u - 1 + (v - 1) * I.getWidth()])
        if (!connexe(I, u - 1, v - 1, mean_value, u_cog, v_cog, n, checkTab))
          edge = true;

    if (v >= 1 && u + 1 < I.getWidth())
      if (!checkTab[u + 1 + (v - 1) * I.getWidth()])
        if (!connexe(I, u + 1, v - 1, mean_value, u_cog, v_cog, n, checkTab))
          edge = true;

    if (v + 1 < I.getHeight() && u >= 1)
      if (!checkTab[u - 1 + (v + 1) * I.getWidth()])
        if (!connexe(I, u - 1, v + 1, mean_value, u_cog, v_cog, n, checkTab))
          edge = true;

    if (v + 1 < I.getHeight() && u + 1 < I.getWidth())
      if (!checkTab[u + 1 + (v + 1) * I.getWidth()])
        if (!connexe(I, u + 1, v + 1, mean_value, u_cog, v_cog, n, checkTab))
          edge = true;
  }

  if (edge) {
    ip_edges_list.push_back(ip);
    if (graphics == true) {
      vpImagePoint ip_(ip);
      for (unsigned int t = 0; t < thickness; t++) {
        ip_.set_u(ip.get_u() + t);
        vpDisplay::displayPoint(I, ip_, vpColor::red);
      }
      // vpDisplay::flush(I);
    }
  }

  return true;
}

/*!

  Compute the center of gravity (COG) of the dot using connex
  components.  We assume the origin pixel (u, v) is in the dot. If
  not, the dot is seach around this origin using a spiral search.

  \param I : Image to process.
  \param u : Starting pixel coordinate along the columns from where the
  dot is searched .

  \param v : Starting pixel coordinate along the rows from where the
  dot is searched .

  \warning The content of the image is modified.

  \exception vpTrackingException::featureLostError : If the tracking fails.

  \sa connexe()
*/
void vpDot::COG(const vpImage<unsigned char> &I, double &u, double &v)
{
  // Set the maximal number of points considering the maximal dot size
  // image percentage
  nbMaxPoint = (I.getWidth() * I.getHeight()) * maxDotSizePercentage;

  // segmentation de l'image apres seuillage
  // (etiquetage des composante connexe)
  if (compute_moment)
    m00 = m11 = m02 = m20 = m10 = m01 = mu11 = mu20 = mu02 = 0;

  double u_cog = 0;
  double v_cog = 0;
  double npoint = 0;
  this->mean_gray_level = 0;

  ip_connexities_list.clear();
  ip_edges_list.clear();

  // Initialise the boundig box
  this->u_min = I.getWidth();
  this->u_max = 0;
  this->v_min = I.getHeight();
  this->v_max = 0;

#if 0
  // Original version
  if (  connexe(I, (unsigned int)u, (unsigned int)v,
		gray_level_min, gray_level_max,
		mean_gray_level, u_cog, v_cog, npoint) == vpDot::out)
  {
    bool sol = false ;
    unsigned int pas  ;
    for (pas = 2 ; pas <= 25 ; pas ++ )if (sol==false)
    {
      for (int k=-1 ; k <=1 ; k++) if (sol==false)
	for (int l=-1 ; l <=1 ; l++) if (sol==false)
	{
	  u_cog = 0 ;
	  v_cog = 0 ;
    ip_connexities_list.clear() ;
	 
	  this->mean_gray_level = 0 ;
	  if (connexe(I, (unsigned int)(u+k*pas),(unsigned int)(v+l*pas),
		      gray_level_min, gray_level_max,
		      mean_gray_level, u_cog, v_cog, npoint) != vpDot::out)
	  {
	    sol = true ; u += k*pas ; v += l*pas ;
	  }
	}
    }
    if (sol == false)
    {
      //vpERROR_TRACE("Dot has been lost") ;
      throw(vpTrackingException(vpTrackingException::featureLostError,
				"Dot has been lost")) ;
    }
  }
#else
  // If the dot is not found, search around using a spiral
  if (!connexe(I, (unsigned int)u, (unsigned int)v, mean_gray_level, u_cog, v_cog, npoint)) {
    bool sol = false;

    unsigned int right = 1;
    unsigned int botom = 1;
    unsigned int left = 2;
    unsigned int up = 2;
    double u_ = u, v_ = v;
    unsigned int k;

    // Spiral search from the center to find the nearest dot
    while ((right < SPIRAL_SEARCH_SIZE) && (sol == false)) {
      for (k = 1; k <= right; k++)
        if (sol == false) {
          u_cog = 0;
          v_cog = 0;
          ip_connexities_list.clear();
          ip_edges_list.clear();

          this->mean_gray_level = 0;
          if (connexe(I, (unsigned int)u_ + k, (unsigned int)(v_), mean_gray_level, u_cog, v_cog, npoint)) {
            sol = true;
            u = u_ + k;
            v = v_;
          }
        }
      u_ += k;
      right += 2;

      for (k = 1; k <= botom; k++)
        if (sol == false) {
          u_cog = 0;
          v_cog = 0;
          ip_connexities_list.clear();
          ip_edges_list.clear();

          this->mean_gray_level = 0;

          if (connexe(I, (unsigned int)(u_), (unsigned int)(v_ + k), mean_gray_level, u_cog, v_cog, npoint)) {
            sol = true;
            u = u_;
            v = v_ + k;
          }
        }
      v_ += k;
      botom += 2;

      for (k = 1; k <= left; k++)
        if (sol == false) {
          u_cog = 0;
          v_cog = 0;
          ip_connexities_list.clear();
          ip_edges_list.clear();

          this->mean_gray_level = 0;

          if (connexe(I, (unsigned int)(u_ - k), (unsigned int)(v_), mean_gray_level, u_cog, v_cog, npoint)) {
            sol = true;
            u = u_ - k;
            v = v_;
          }
        }
      u_ -= k;
      left += 2;

      for (k = 1; k <= up; k++)
        if (sol == false) {
          u_cog = 0;
          v_cog = 0;
          ip_connexities_list.clear();
          ip_edges_list.clear();

          this->mean_gray_level = 0;

          if (connexe(I, (unsigned int)(u_), (unsigned int)(v_ - k), mean_gray_level, u_cog, v_cog, npoint)) {
            sol = true;
            u = u_;
            v = v_ - k;
          }
        }
      v_ -= k;
      up += 2;
    }

    if (sol == false) {
      // vpERROR_TRACE("Dot has been lost") ;
      throw(vpTrackingException(vpTrackingException::featureLostError, "Dot has been lost"));
    }
  }

#endif
  /*
    vpImagePoint ip;
    unsigned int i, j;
    std::list<vpImagePoint>::iterator it;
    for (it = ip_connexities_list.begin(); it != ip_connexities_list.end(); it
    ++) { ip = *it; i = (unsigned int) ip.get_i(); j = (unsigned int)
    ip.get_j(); I[i][j] = 255 ;
    }*/

  u_cog = u_cog / npoint;
  v_cog = v_cog / npoint;

  u = u_cog;
  v = v_cog;

  // Initialize the threshold for the next call to track()
  double Ip = pow((double)this->mean_gray_level / 255, 1 / gamma);

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

  // vpCTRACE << "gray_level_min: " << gray_level_min << std::endl;
  // vpCTRACE << "gray_level_max: " << gray_level_max << std::endl;

  if (npoint < 5) {
    // vpERROR_TRACE("Dot to small") ;
    throw(vpTrackingException(vpTrackingException::featureLostError, "Dot to small"));
  }

  if (npoint > nbMaxPoint) {
    //    vpERROR_TRACE("Too many point %lf (%lf%%). Max allowed is %lf
    //    (%lf%%). This threshold can be modified using the setMaxDotSize()
    //    method.",
    //		  npoint, npoint / (I.getWidth() * I.getHeight()),
    //		  nbMaxPoint, maxDotSizePercentage) ;

    throw(vpTrackingException(vpTrackingException::featureLostError,
                              "Too many point %lf (%lf%%). Max allowed is "
                              "%lf (%lf%%). This threshold can be modified "
                              "using the setMaxDotSize() method.",
                              npoint, npoint / (I.getWidth() * I.getHeight()), nbMaxPoint, maxDotSizePercentage));
  }
}

/*!
  Maximal size of the region to track in terms of image size percentage.

  \param percentage : Image size percentage corresponding to the dot
  maximal size. Values should be in ]0 : 1]. If 1, that means that the
  dot to track could take up the whole image.

  During the tracking, if the dot size if bigger than the maximal size
  allowed an exception is throwed :
  vpTrackingException::featureLostError.

*/
void vpDot::setMaxDotSize(double percentage)
{
  if (percentage <= 0.0 || percentage > 1.0) {
    // print a warning. We keep the default percentage
    vpTRACE("Max dot size percentage is requested to be set to %lf.",
            "Value should be in ]0:1]. Value will be set to %lf.", percentage, maxDotSizePercentage);
  } else {
    maxDotSizePercentage = percentage;
  }
}

/*!

  Initialize the tracking with a mouse click and update the dot
  characteristics (center of gravity, moments).

  Wait a user click in a gray area in the image I. The clicked pixel
  will be the starting point from which the dot will be tracked.

  The threshold used to segment the dot is set with the grayLevelPrecision
  parameter. See the formula in setGrayLevelPrecision() function.

  The sub pixel coordinates of the dot are updated. To get the center
  of gravity coordinates of the dot, use getCog(). To
  compute the moments use setComputeMoments(true) before a call to
  initTracking().

  \warning The content of the image modified since we call track() to
  compute the dot characteristics.

  \param I : Image to process.

  \sa track(), getCog()
*/
void vpDot::initTracking(const vpImage<unsigned char> &I)
{
  while (vpDisplay::getClick(I, cog) != true)
    ;

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

  try {
    track(I);
  } catch (const vpException &e) {
    throw(e);
  }
}

/*!

  Initialize the tracking for a dot supposed to be located at (u,v) and
  updates the dot characteristics (center of gravity, moments).

  The threshold used to segment the dot is set to 80 percent of the
  gray level of the pixel (u,v).

  The sub pixel coordinates of the dot are updated. To get the center
  of gravity coordinates of the dot, use getCog(). To
  compute the moments use setComputeMoments(true) before a call to
  initTracking().

  \warning The content of the image modified since we call track() to
  compute the dot characteristics.

  \param I : Image to process.

  \param ip : Location of the starting point from which the dot will be
  tracked in the image.

  \sa track()
*/
void vpDot::initTracking(const vpImage<unsigned char> &I, const vpImagePoint &ip)
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
  try {
    track(I);
  } catch (const vpException &e) {
    throw(e);
  }
}

/*!

  Initialize the tracking for a dot supposed to be located at (u,v) and
  updates the dot characteristics (center of gravity, moments).

  The sub pixel coordinates of the dot are updated. To get the center
  of gravity coordinates of the dot, use getCog(). To
  compute the moments use setComputeMoments(true) before a call to
  initTracking().

  \warning The content of the image modified since we call track() to
  compute the dot characteristics.

  \param I : Image to process.

  \param ip : Location of the starting point from which the dot will
  be tracked in the image.

  \param level_min : Minimum gray level threshold used to segment the dot;
  value comprised between 0 and 255.

  \param level_max : Maximum gray level threshold used to segment the
  dot; value comprised between 0 and 255. \e gray_level_max should be
  greater than \e gray_level_min.

  \sa track(), getCog()
*/
void vpDot::initTracking(const vpImage<unsigned char> &I, const vpImagePoint &ip, unsigned int level_min,
                         unsigned int level_max)
{

  cog = ip;

  this->gray_level_min = level_min;
  this->gray_level_max = level_max;

  try {
    track(I);
  } catch (const vpException &e) {
    throw(e);
  }
}

/*!
  Track and compute the dot characteristics.

  To get the center of gravity coordinates of the dot, use
  getCog(). To compute the moments use setComputeMoments(true) before
  a call to initTracking().

  \warning The image is modified (all the pixels that belong to the point
  are set to white (ie to 255).

  \param I : Image to process.

  \sa getCog()
*/
void vpDot::track(const vpImage<unsigned char> &I)
{
  try {
    setGrayLevelOut();
    double u = this->cog.get_u();
    double v = this->cog.get_v();

    COG(I, u, v);

    this->cog.set_u(u);
    this->cog.set_v(v);

    if (compute_moment == true) {
      mu11 = m11 - u * m01;
      mu02 = m02 - v * m01;
      mu20 = m20 - u * m10;
    }

    if (graphics) {
      // display a red cross at the center of gravity's location in the image.
      vpDisplay::displayCross(I, this->cog, 3 * thickness + 8, vpColor::red, thickness);
    }

  } catch (const vpException &e) {
    throw(e);
  }
}

/*!

  Track and updates the new dot coordinates

  To compute the moments use setComputeMoments(true) before a call to
  initTracking() or track().

  \warning The image is modified (all the pixels that belong to the point
  are set to white (ie to 255).

  \param I : Image to process.

  \param ip [out] : Sub pixel coordinate of the tracked dot center of gravity.
*/
void vpDot::track(const vpImage<unsigned char> &I, vpImagePoint &ip)
{
  track(I);

  ip = this->cog;
}

/*!
  Display in overlay the dot edges and center of gravity.

  \param I : Image.
  \param color : The color used for the display.
  \param thick : Thickness of the displayed cross located at the dot cog.
*/
void vpDot::display(const vpImage<unsigned char> &I, vpColor color, unsigned int thick) const
{
  vpDisplay::displayCross(I, cog, 3 * thickness + 8, color, thick);
  std::list<vpImagePoint>::const_iterator it;

  for (it = ip_edges_list.begin(); it != ip_edges_list.end(); ++it) {
    vpDisplay::displayPoint(I, *it, color);
  }
}

/*!

  Set the precision of the gray level of the dot.

  \param precision : It is a double precision float which value is
  in ]0,1]:
  - 1 means full precision, whereas values close to 0 show a very bad
  accuracy.
  - Values lower or equal to 0 are brought back to an epsion>0
  - Values higher than  1 are brought back to 1
  If the initial gray level is I, the gray levels of the dot will be between :
  \f$Imin=255*\big((\frac{I}{255})^{{\gamma}^{-1}}-(1-grayLevelPrecision)\big)^{\gamma}\f$
  and
  \f$Imax=255*\big((\frac{I}{255})^{{\gamma}^{-1}}+(1-grayLevelPrecision)\big)^{\gamma}\f$
  with \f$\gamma=1.5\f$ .

  \sa setWidth(), setHeight(), setGrayLevelMin(), setGrayLevelMax()
*/
void vpDot::setGrayLevelPrecision(const double &precision)
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

  Display the dot center of gravity and its list of edges.

  \param I : The image used as background.

  \param cog : The center of gravity.

  \param edges_list : The list of edges;

  \param color : Color used to display the dot.

  \param thickness : Thickness of the dot.
*/
void vpDot::display(const vpImage<unsigned char> &I, const vpImagePoint &cog, const std::list<vpImagePoint> &edges_list,
                    vpColor color, unsigned int thickness)
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
void vpDot::display(const vpImage<vpRGBa> &I, const vpImagePoint &cog, const std::list<vpImagePoint> &edges_list,
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
VISP_EXPORT std::ostream &operator<<(std::ostream &os, vpDot &d) { return (os << "(" << d.getCog() << ")"); };
