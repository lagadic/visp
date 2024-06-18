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

/*
  \file vpDot.cpp
  \brief Track a white dot
*/

#include <visp3/blob/vpDot.h>
#include <visp3/core/vpColor.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpTrackingException.h>

#include <vector>

BEGIN_VISP_NAMESPACE

/*
  \class vpDot
  \brief Track a white dot
*/

/* spiral size for the dot search */
const unsigned int vpDot::SPIRAL_SEARCH_SIZE = 350;

/*!
  Initialize the tracker with default parameters.
  - connexity type is set to 4 (see setConnexityType())
  - dot maximal size is set to 25% of the image size (see setMaxDotSize())
*/
void vpDot::init()
{
  const unsigned int val_max = 255;
  const unsigned int val_median = 128;
  m_cog.set_u(0);
  m_cog.set_v(0);

  m_compute_moment = false;
  m_graphics = false;
  m_thickness = 1;
  m_maxDotSizePercentage = 0.25; // 25 % of the image size

  m_mean_gray_level = 0;
  m_gray_level_min = val_median;
  m_gray_level_max = val_max;
  m_grayLevelPrecision = 0.85;
  m_gamma = 1.5;

  m00 = 0;
  m11 = 0;
  m02 = 0;
  m20 = 0;
  m10 = 0;
  m01 = 0;
  mu11 = 0;
  mu02 = 0;
  mu20 = 0;

  m_connexityType = CONNEXITY_4;

  m_u_min = 0;
  m_u_max = 0;
  m_v_min = 0;
  m_v_max = 0;

  m_gray_level_out = 0;
  m_nbMaxPoint = 0;
}

vpDot::vpDot()
  : m00(0.), m01(0.), m10(0.), m11(0.), m20(0.), m02(0.), mu11(0.), mu20(0.), mu02(0.), m_ip_connexities_list(),
  m_ip_edges_list(), m_connexityType(CONNEXITY_4), m_cog(), m_u_min(0), m_u_max(0), m_v_min(0), m_v_max(0), m_graphics(false),
  m_thickness(1), m_maxDotSizePercentage(0.25), m_gray_level_out(0), m_mean_gray_level(0),
  m_grayLevelPrecision(0.85), m_gamma(1.5), m_compute_moment(false), m_nbMaxPoint(0)
{
  const unsigned int val_min = 128;
  const unsigned int val_max = 255;
  m_gray_level_min = val_min;
  m_gray_level_max = val_max;
}

/*!
  \brief Constructor with initialization of the dot location.

  \param cog : An image point with sub-pixel coordinates corresponding to the blob center of gravity.
 */
vpDot::vpDot(const vpImagePoint &cog)
  : m00(0.), m01(0.), m10(0.), m11(0.), m20(0.), m02(0.), mu11(0.), mu20(0.), mu02(0.), m_ip_connexities_list(),
  m_ip_edges_list(), m_connexityType(CONNEXITY_4), m_cog(cog), m_u_min(0), m_u_max(0), m_v_min(0), m_v_max(0), m_graphics(false),
  m_thickness(1), m_maxDotSizePercentage(0.25), m_gray_level_out(0), m_mean_gray_level(0),
  m_grayLevelPrecision(0.85), m_gamma(1.5), m_compute_moment(false), m_nbMaxPoint(0)
{
  const unsigned int val_min = 128;
  const unsigned int val_max = 255;
  m_gray_level_min = val_min;
  m_gray_level_max = val_max;
}

/*!
  \brief Copy constructor.
 */
vpDot::vpDot(const vpDot &d)
  : vpTracker(d), m00(0.), m01(0.), m10(0.), m11(0.), m20(0.), m02(0.), mu11(0.), mu20(0.), mu02(0.),
  m_ip_connexities_list(), m_ip_edges_list(), m_connexityType(CONNEXITY_4), m_cog(), m_u_min(0), m_u_max(0), m_v_min(0), m_v_max(0),
  m_graphics(false), m_thickness(1), m_maxDotSizePercentage(0.25), m_gray_level_out(0), m_mean_gray_level(0),
  m_grayLevelPrecision(0.85), m_gamma(1.5), m_compute_moment(false), m_nbMaxPoint(0)
{

  *this = d;
}

/*!
  \brief Destructor.
 */
vpDot::~vpDot() { m_ip_connexities_list.clear(); }

/*!
  \brief Copy operator.
 */
vpDot &vpDot::operator=(const vpDot &d)
{
  m_ip_edges_list = d.m_ip_edges_list;
  m_ip_connexities_list = d.m_ip_connexities_list;
  m_connexityType = d.m_connexityType;
  m_cog = d.getCog();

  m_u_min = d.m_u_min;
  m_v_min = d.m_v_min;
  m_u_max = d.m_u_max;
  m_v_max = d.m_v_max;

  m_graphics = d.m_graphics;
  m_thickness = d.m_thickness;
  m_maxDotSizePercentage = d.m_maxDotSizePercentage;
  m_gray_level_out = d.m_gray_level_out;
  m_mean_gray_level = d.m_mean_gray_level;
  m_gray_level_min = d.m_gray_level_min;
  m_gray_level_max = d.m_gray_level_max;
  m_grayLevelPrecision = d.m_grayLevelPrecision;
  m_gamma = d.m_gamma;
  m_compute_moment = d.m_compute_moment;
  m_nbMaxPoint = d.m_nbMaxPoint;

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

bool vpDot::operator!=(const vpDot &d) const { return (m_cog != d.getCog()); }

bool vpDot::operator==(const vpDot &d) const { return (m_cog == d.getCog()); }

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
  if (m_gray_level_min == 0) {
    const unsigned int val_max = 255;
    if (m_gray_level_max == val_max) {
      // m_gray_level_min = 0 and m_gray_level_max = 255: this should not occur
      throw(vpTrackingException(vpTrackingException::initializationError, "Unable to choose a good \"out\" level"));
    }
    m_gray_level_out = static_cast<unsigned char>(m_gray_level_max + 1u);
  }
}

/*!
  Perform the tracking of a dot by connex components.

  \param mean_value : Threshold to use for the next call to track()
  and corresponding to the mean value of the dot intensity.

  \warning The content of the image is modified thanks to
  setGrayLevelOut() called before. This method choose a gray level
  (default is 0) used to modify the "in" dot level in "out" dot
  level. This gray level is here needed to stop the recursivity. The
  pixels of the dot are set to this new gray level "\out\".

  \return vpDot::out if an error occurs, vpDot::in otherwise.

  \sa setGrayLevelOut()
*/
bool vpDot::connexe(const vpImage<unsigned char> &I, unsigned int u, unsigned int v, double &mean_value,
                    vpImagePoint &uv_cog, unsigned int &npoints)
{
  std::vector<bool> checkTab(I.getWidth() * I.getHeight(), false);
  return connexe(I, u, v, mean_value, uv_cog, npoints, checkTab);
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
bool vpDot::connexe(const vpImage<unsigned char> &I, unsigned int u, unsigned int v, double &mean_value,
                    vpImagePoint &uv_cog, unsigned int &npoints, std::vector<bool> &checkTab)
{

  unsigned int width = I.getWidth();
  unsigned int height = I.getHeight();

  // Test if we are in the image
  if ((u >= width) || (v >= height)) {
    return false;
  }

  if (checkTab[u + (v * I.getWidth())]) {
    return true;
  }

  vpImagePoint ip;
  ip.set_u(u);
  ip.set_v(v);

  if ((I[v][u] >= m_gray_level_min) && (I[v][u] <= m_gray_level_max)) {
    checkTab[(v * I.getWidth()) + u] = true;

    m_ip_connexities_list.push_back(ip);

    uv_cog.set_u(uv_cog.get_u() + u);
    uv_cog.set_v(uv_cog.get_v() + v);
    ++npoints;

    if (npoints > m_nbMaxPoint) {
      throw(vpTrackingException(vpTrackingException::featureLostError,
                                "Too many point %u (%f%% of image size). "
                                "This threshold can be modified using the setMaxDotSize() "
                                "method.",
                                npoints, static_cast<float>(npoints) / (I.getWidth() * I.getHeight()), m_nbMaxPoint, m_maxDotSizePercentage));
    }

    // Bounding box update
    if (u < m_u_min) {
      m_u_min = u;
    }
    if (u > m_u_max) {
      m_u_max = u;
    }
    if (v < m_v_min) {
      m_v_min = v;
    }
    if (v > m_v_max) {
      m_v_max = v;
    }

    // Mean value of the dot intensities
    mean_value = ((mean_value * (npoints - 1)) + I[v][u]) / npoints;
    if (m_compute_moment == true) {
      ++m00;
      m10 += u;
      m01 += v;
      m11 += (u * v);
      m20 += u * u;
      m02 += v * v;
    }
  }
  else {
    return false;
  }

  bool edge = false;

  if (u >= 1) {
    if (!checkTab[(u - 1) + (v * I.getWidth())]) {
      if (!connexe(I, u - 1, v, mean_value, uv_cog, npoints, checkTab)) {
        edge = true;
      }
    }
  }

  if ((u + 1) < I.getWidth()) {
    if (!checkTab[u + 1 + (v * I.getWidth())]) {
      if (!connexe(I, u + 1, v, mean_value, uv_cog, npoints, checkTab)) {
        edge = true;
      }
    }
  }

  if (v >= 1) {
    if (!checkTab[u + ((v - 1) * I.getWidth())]) {
      if (!connexe(I, u, v - 1, mean_value, uv_cog, npoints, checkTab)) {
        edge = true;
      }
    }
  }

  if ((v + 1) < I.getHeight()) {
    if (!checkTab[u + ((v + 1) * I.getWidth())]) {
      if (!connexe(I, u, v + 1, mean_value, uv_cog, npoints, checkTab)) {
        edge = true;
      }
    }
  }

  if (m_connexityType == CONNEXITY_8) {
    if ((v >= 1) && (u >= 1)) {
      if (!checkTab[(u - 1) + ((v - 1) * I.getWidth())]) {
        if (!connexe(I, u - 1, v - 1, mean_value, uv_cog, npoints, checkTab)) {
          edge = true;
        }
      }
    }

    if ((v >= 1) && ((u + 1) < I.getWidth())) {
      if (!checkTab[u + 1 + ((v - 1) * I.getWidth())]) {
        if (!connexe(I, u + 1, v - 1, mean_value, uv_cog, npoints, checkTab)) {
          edge = true;
        }
      }
    }

    if (((v + 1) < I.getHeight()) && (u >= 1)) {
      if (!checkTab[(u - 1) + ((v + 1) * I.getWidth())]) {
        if (!connexe(I, u - 1, v + 1, mean_value, uv_cog, npoints, checkTab)) {
          edge = true;
        }
      }
    }

    if (((v + 1) < I.getHeight()) && ((u + 1) < I.getWidth())) {
      if (!checkTab[u + 1 + ((v + 1) * I.getWidth())]) {
        if (!connexe(I, u + 1, v + 1, mean_value, uv_cog, npoints, checkTab)) {
          edge = true;
        }
      }
    }
  }

  if (edge) {
    m_ip_edges_list.push_back(ip);
    if (m_graphics == true) {
      vpImagePoint ip_(ip);
      for (unsigned int t = 0; t < m_thickness; ++t) {
        ip_.set_u(ip.get_u() + t);
        vpDisplay::displayPoint(I, ip_, vpColor::red);
      }
      // --comment: use vpDisplay to flush I
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
  m_nbMaxPoint = (I.getWidth() * I.getHeight()) * m_maxDotSizePercentage;

  // segmentation de l'image apres seuillage
  // (etiquetage des composante connexe)
  if (m_compute_moment) {
    m00 = 0;
    m11 = 0;
    m02 = 0;
    m20 = 0;
    m10 = 0;
    m01 = 0;
    mu11 = 0;
    mu20 = 0;
    mu02 = 0;
  }

  vpImagePoint uv_cog(0, 0);
  unsigned int npoints = 0;
  m_mean_gray_level = 0;

  m_ip_connexities_list.clear();
  m_ip_edges_list.clear();

  // Initialise the bounding box
  m_u_min = I.getWidth();
  m_u_max = 0;
  m_v_min = I.getHeight();
  m_v_max = 0;

  // If the dot is not found, search around using a spiral
  if (!connexe(I, static_cast<unsigned int>(u), static_cast<unsigned int>(v), m_mean_gray_level, uv_cog, npoints)) {
    bool sol = false;

    unsigned int right = 1;
    unsigned int botom = 1;
    unsigned int left = 2;
    unsigned int up = 2;
    double u_ = u, v_ = v;
    unsigned int k;
    const unsigned int val_2 = 2;
    // Spiral search from the center to find the nearest dot
    while ((right < SPIRAL_SEARCH_SIZE) && (sol == false)) {
      for (k = 1; k <= right; ++k) {
        if (sol == false) {
          uv_cog.set_uv(0, 0);
          m_ip_connexities_list.clear();
          m_ip_edges_list.clear();

          m_mean_gray_level = 0;
          if (connexe(I, static_cast<unsigned int>(u_) + k, static_cast<unsigned int>(v_), m_mean_gray_level, uv_cog, npoints)) {
            sol = true;
            u = u_ + k;
            v = v_;
          }
        }
      }
      u_ += k;
      right += val_2;

      for (k = 1; k <= botom; ++k) {
        if (sol == false) {
          uv_cog.set_uv(0, 0);
          m_ip_connexities_list.clear();
          m_ip_edges_list.clear();

          m_mean_gray_level = 0;

          if (connexe(I, static_cast<unsigned int>(u_), static_cast<unsigned int>(v_ + k), m_mean_gray_level, uv_cog, npoints)) {
            sol = true;
            u = u_;
            v = v_ + k;
          }
        }
      }
      v_ += k;
      botom += val_2;

      for (k = 1; k <= left; ++k) {
        if (sol == false) {
          uv_cog.set_uv(0, 0);
          m_ip_connexities_list.clear();
          m_ip_edges_list.clear();

          m_mean_gray_level = 0;

          if (connexe(I, static_cast<unsigned int>(u_ - k), static_cast<unsigned int>(v_), m_mean_gray_level, uv_cog, npoints)) {
            sol = true;
            u = u_ - k;
            v = v_;
          }
        }
      }
      u_ -= k;
      left += val_2;

      for (k = 1; k <= up; ++k) {
        if (sol == false) {
          uv_cog.set_uv(0, 0);
          m_ip_connexities_list.clear();
          m_ip_edges_list.clear();

          m_mean_gray_level = 0;

          if (connexe(I, static_cast<unsigned int>(u_), static_cast<unsigned int>(v_ - k), m_mean_gray_level, uv_cog, npoints)) {
            sol = true;
            u = u_;
            v = v_ - k;
          }
        }
      }
      v_ -= k;
      up += val_2;
    }

    if (sol == false) {
      throw(vpTrackingException(vpTrackingException::featureLostError, "Dot has been lost"));
    }
  }

  uv_cog.set_u(uv_cog.get_u() / npoints);
  uv_cog.set_v(uv_cog.get_v() / npoints);

  u = uv_cog.get_u();
  v = uv_cog.get_v();

  const unsigned int val_max = 255;
  // Initialize the threshold for the next call to track()
  double Ip = pow(static_cast<double>(m_mean_gray_level) / val_max, 1 / m_gamma);

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

  const double nbMinPoint = 5;
  if (npoints < nbMinPoint) {
    throw(vpTrackingException(vpTrackingException::featureLostError, "Dot to small"));
  }

  if (npoints > m_nbMaxPoint) {
    throw(vpTrackingException(vpTrackingException::featureLostError,
                              "Too many point %lf (%lf%%). Max allowed is "
                              "%u (%f%%). This threshold can be modified "
                              "using the setMaxDotSize() method.",
                              npoints, static_cast<float>(npoints) / (I.getWidth() * I.getHeight()), m_nbMaxPoint, m_maxDotSizePercentage));
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
  if ((percentage <= 0.0) || (percentage > 1.0)) {
    // print a warning. We keep the default percentage
    std::cout << "Max dot size percentage is requested to be set to " << percentage << "." << std::endl;
    std::cout << "Value should be in ]0:1]. Value will be set to " << m_maxDotSizePercentage << "." << std::endl;
  }
  else {
    m_maxDotSizePercentage = percentage;
  }
}

/*!

  Initialize the tracking with a mouse click and update the dot
  characteristics (center of gravity, moments).

  Wait a user click in a gray area in the image I. The clicked pixel
  will be the starting point from which the dot will be tracked.

  The threshold used to segment the dot is set with the gray level precision
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
  while (vpDisplay::getClick(I, m_cog) != true) {
    // Wait until a click is detected
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

  track(I);
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

  track(I);
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
  m_cog = ip;

  m_gray_level_min = level_min;
  m_gray_level_max = level_max;

  track(I);
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
  setGrayLevelOut();
  double u = m_cog.get_u();
  double v = m_cog.get_v();

  COG(I, u, v);

  m_cog.set_u(u);
  m_cog.set_v(v);

  if (m_compute_moment == true) {
    mu11 = m11 - (u * m01);
    mu02 = m02 - (v * m01);
    mu20 = m20 - (u * m10);
  }

  if (m_graphics) {
    const unsigned int val_3 = 3;
    const unsigned int val_8 = 8;
    // display a red cross at the center of gravity's location in the image.
    vpDisplay::displayCross(I, m_cog, (val_3 * m_thickness) + val_8, vpColor::red, m_thickness);
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

  ip = m_cog;
}

/*!
  Display in overlay the dot edges and center of gravity.

  \param I : Image.
  \param color : The color used for the display.
  \param thick : Thickness of the displayed cross located at the dot cog.
*/
void vpDot::display(const vpImage<unsigned char> &I, vpColor color, unsigned int thick) const
{
  const unsigned int val_3 = 3;
  const unsigned int val_8 = 8;
  vpDisplay::displayCross(I, m_cog, (val_3 * m_thickness) + val_8, color, thick);
  std::list<vpImagePoint>::const_iterator it;

  std::list<vpImagePoint>::const_iterator m_ip_edges_list_end = m_ip_edges_list.end();
  for (it = m_ip_edges_list.begin(); it != m_ip_edges_list_end; ++it) {
    vpDisplay::displayPoint(I, *it, color);
  }
}

/*!

  Set the precision of the gray level of the dot.

  \param precision : It is a double precision float which value is
  in ]0,1]:
  - 1 means full precision, whereas values close to 0 show a very bad
  accuracy.
  - Values lower or equal to 0 are brought back to an epsilon>0
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
void vpDot::display(const vpImage<vpRGBa> &I, const vpImagePoint &cog, const std::list<vpImagePoint> &edges_list,
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
VISP_EXPORT std::ostream &operator<<(std::ostream &os, vpDot &d) { return (os << "(" << d.getCog() << ")"); };

END_VISP_NAMESPACE
