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
 * Moving edges.
 */

/*!
 * \file vpMeSite.h
 * \brief Moving edges
 */

#ifndef VP_ME_SITE_H
#define VP_ME_SITE_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/me/vpMe.h>

BEGIN_VISP_NAMESPACE

/*!
 * \class vpMeSite
 * \ingroup module_me
 *
 * \brief Performs search in a given direction(normal) for a given
 *  distance(pixels) for a given 'site'. Gives the most likely site
 *  given the probability from an ME mask
 *
 * - Bug fix: rewrote application of masks to use the temporal
 *   information instead of applying both temporal masks to the same
 *   image. ie: spatial -> spatio/temporal
 *
 * - Added new tracking function to choose the most similar edge
 *   amongst all edges found.
 *
 * - sample step.
*/
class VISP_EXPORT vpMeSite
{
public:
  /*!
   * Type moving-edges site of display.
   */
  typedef enum
  {
    NONE, //!< Not displayed
    RANGE, //!<
    RESULT, //!<
    RANGE_RESULT //!<
  } vpMeSiteDisplayType;

  /*!
   * Moving-edge site state
   */
  typedef enum
  {
    NO_SUPPRESSION = 0,   ///< Point successfully tracked.
    CONTRAST = 1,         ///< Point not tracked due to a contrast problem, but retained in the ME list.
#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
    CONSTRAST = CONTRAST, ///< Deprecated. Point not tracked due to a likelihood problem, but retained in the ME list. Use instead CONTRAST.
#endif
    THRESHOLD = 2,        ///< Point not tracked due to the likelihood that is below the threshold, but retained in the ME list.
    M_ESTIMATOR = 3,      ///< Point detected as an outlier during virtual visual-servoing.
    TOO_NEAR = 4,         ///< Point not tracked anymore, since too near from its neighbor.
    UNKNOW = 5,           ///< Reserved.
    OUTSIDE_ROI_MASK = 6  ///< Point is outside the region of interest mask, but retained in the ME list.
  } vpMeSiteState;

  //! Integer coordinate along i of a site
  int m_i;
  //! Integer coordinates along j of a site
  int m_j;
  //! Subpixel coordinates along i of a site
  double m_ifloat;
  //! Subpixel coordinates along j of a site
  double m_jfloat;
  //! Mask sign
  int m_mask_sign;
  //! Angle of tangent at site
  double m_alpha;
  //! Convolution of Site in previous image
  double m_convlt;
  //! Convolution of Site in previous image
  double m_normGradient;
  //! Uncertainty of point given as a probability between 0 and 1
  double m_weight;
  //! Old likelihood ratio threshold (to be avoided) or easy-to-use normalized threshold: minimal contrast
  double m_contrastThreshold;

public:
  /*!
   * Default constructor.
   */
  vpMeSite();

  /*!
   * Constructor from pixel coordinates.
   */
  vpMeSite(const double &ip, const double &jp);

  /*!
   * Copy constructor.
   */
  vpMeSite(const vpMeSite &mesite);

  /*!
   * Destructor.
   */
  virtual ~vpMeSite() { };

  /*!
   * Compute convolution.
   */
  double convolution(const vpImage<unsigned char> &ima, const vpMe *me);

  /*!
   * Display moving edges in image I.
   * @param I : Input image.
   */
  void display(const vpImage<unsigned char> &I);

  /*!
   * Display moving edges in image I.
   * @param I : Input image.
   */
  void display(const vpImage<vpRGBa> &I);

  /*!
   * Get the angle of tangent at site.
   *
   * \return value of alpha
   */
  inline double getAlpha() const { return m_alpha; }

  /*!
   * Return site weight or uncertainty as a probability between 0 and 1.
   */
  inline double getWeight() const { return m_weight; }

  /*!
   * Construct and return the list of vpMeSite along the normal to the contour,
   * in the given range.
   * \pre : Subpixel coordinates (ifloat, jfloat) and the direction of the normal (alpha) have to be set.
   * \param I : Image in which the display is performed.
   * \param range :  +/- the range within which the pixel's correspondent will be sought.
   * \return Pointer to the list of query sites
   */
  vpMeSite *getQueryList(const vpImage<unsigned char> &I, const int &range) const;

  /*!
   * Return integer coordinate along i (rows).
   * \sa get_ifloat()
   */
  inline int get_i() const { return m_i; }

  /*!
   * Return integer coordinate along j (columns).
   * \sa get_jfloat()
   */
  inline int get_j() const { return m_j; }

  /*!
   * Return subpixel coordinate along i (rows).
   * \sa get_i()
   */
  inline double get_ifloat() const { return m_ifloat; }

  /*!
   * Return subpixel coordinate along j (columns).
   * \sa get_j()
   */
  inline double get_jfloat() const { return m_jfloat; }

  /*!
   * Initialize moving-edge site with default parameters.
   */
  void init();

  /*!
   * Initialize moving-edge site parameters.
   */
  void init(const double &ip, const double &jp, const double &alphap);

  /*!
   * Initialize moving-edge site parameters.
   */
  void init(const double &ip, const double &jp, const double &alphap, const double &convltp);

  /*!
   * Initialize moving-edge site parameters.
   */
  void init(const double &ip, const double &jp, const double &alphap, const double &convltp, const int &sign);

  /*!
   * Initialize moving-edge site parameters.
   */
  void init(const double &ip, const double &jp, const double &alphap, const double &convltp, const int &sign, const double &contrastThreshold);

  /*!
   * Specific function for moving-edges.
   *
   * \warning To display the moving edges graphics a call to vpDisplay::flush() is needed after this function.
   * \param[in] I : Input image.
   * \param[in] me : Pointer to the moving-edges settings.
   * \param[in] test_contrast : When true tracking is based on contrast. The retained moving-edge is the one with
   * similar contrast. When false, the tracking is based on the likelihood. The retained moving-edge is the one with
   * the maximum likelihood.
   */
  void track(const vpImage<unsigned char> &I, const vpMe *me, const bool &test_contrast = true);

  /*!
   * Set the angle of tangent at site.
   *
   * \param a : new value of alpha
   */
  void setAlpha(const double &a) { m_alpha = a; }

  /*!
   * Display selector.
   */
  void setDisplay(vpMeSiteDisplayType select) { m_selectDisplay = select; }

  /*!
   * Set the state of the site.
   *
   * \param flag : flag corresponding to vpMeSiteState
   *
   * \sa vpMeSiteState
   */
  void setState(const vpMeSiteState &flag)
  {
    m_state = flag;
  }

  /*!
   * Get the state of the site.
   *
   * \return flag corresponding to vpMeSiteState
   */
  inline vpMeSiteState getState() const { return m_state; }

  /*!
   * Set the weight or uncertainty of the site.
   *
   * \param weight : New value of weight as a probability between 0 and 1.
   */
  void setWeight(const double &weight) { m_weight = weight; }

  /*!
   * Set the contrast threshold of the site.
   * If the \b vpMe::m_useAutomaticThreshold is set to false, the contrast threshold is set to the global
   * value retrieved using vpMe::getThreshold(). This value can be set using vpMe::setThreshold().
   * Otherwise, the contrast threshold will be set to the highest value
   * between \b thresh and the minimum value set by vpMe::setMinThreshold() that could be retrieved using
   * vpMe::getMinThreshold().
   *
   * \param thresh : new value of contrast threshold
   * \param me: moving-edge parameters
   */
  void setContrastThreshold(const double &thresh, const vpMe &me)
  {
    double threshold;
    if (me.getUseAutomaticThreshold()) {
      threshold = std::max(thresh, me.getMinThreshold());
    }
    else {
      threshold = me.getThreshold();
    }

    m_contrastThreshold = threshold;
  }

  /*!
   * Get the contrast threshold of the site.
   *
   * \return value of the contrast threshold of the site.
   */
  inline double getContrastThreshold() const { return m_contrastThreshold; }

  /*!
   * Copy operator.
   */
  vpMeSite &operator=(const vpMeSite &m);

  /*!
   * Comparison operator.
   */
  int operator!=(const vpMeSite &m);

  /*!
   * ostream operator.
   */
  friend VISP_EXPORT std::ostream &operator<<(std::ostream &os, vpMeSite &vpMeS);

  // Static functions
  /*!
    Compute the square root distance between two moving-edges sites
    \f$ |S1 - S2| = \sqrt{(i_1-i_2)^2+(j_1-j_2)^2} \f$

    \param S1 : First site
    \param S2 : Second site

    \return the distance between the two sites.

    \sa sqrDistance()
  */
  static double distance(const vpMeSite &S1, const vpMeSite &S2)
  {
    return (sqrt(sqrDistance(S1, S2)));
  }

  /*!
    Compute the square distance between two moving-edges sites
    \f$ |S1 - S2| = (i_1-i_2)^2+(j_1-j_2)^2 \f$

    \param S1 : First site
    \param S2 : Second site

    \return The square distance between the two sites.

    \sa distance()
  */
  static double sqrDistance(const vpMeSite &S1, const vpMeSite &S2)
  {
    return (vpMath::sqr(S1.m_ifloat - S2.m_ifloat) + vpMath::sqr(S1.m_jfloat - S2.m_jfloat));
  }

  /*!
   * Display the moving edge site with a color corresponding to their state.
   *
   * - If green : The vpMeSite is a good point.
   * - If blue : The point is removed because of the vpMeSite tracking phase (contrast problem).
   * - If purple : The point is removed because of the vpMeSite tracking phase (threshold problem).
   * - If red : The point is removed because of the robust method in the virtual visual servoing (M-Estimator problem).
   * - If cyan : The point is outside the mask, see vpMeTracker::setMask().
   * - Yellow otherwise.
   *
   * \param I : The image.
   * \param i : Pixel i of the site.
   * \param j : Pixel j of the site.
   * \param state : State of the site.
   */
  static void display(const vpImage<unsigned char> &I, const double &i, const double &j,
                      const vpMeSiteState &state = NO_SUPPRESSION);

  /*!
   * Display the moving edge site with a color corresponding to their state.
   *
   * - If green : The vpMeSite is a good point.
   * - If blue : The point is removed because of the vpMeSite tracking phase (contrast problem).
   * - If purple : The point is removed because of the vpMeSite tracking phase (threshold problem).
   * - If red : The point is removed because of the robust method in the virtual visual servoing (M-Estimator problem).
   * - If cyan : The point is outside the mask, see vpMeTracker::setMask().
   * - Yellow otherwise
   *
   * \param I : The image.
   * \param i : Pixel i of the site.
   * \param j : Pixel j of the site.
   * \param state : State of the site.
   */
  static void display(const vpImage<vpRGBa> &I, const double &i, const double &j,
                      const vpMeSiteState &state = NO_SUPPRESSION);

private:
  vpMeSiteDisplayType m_selectDisplay; //!< Display selector
  vpMeSiteState m_state; //!< Site state
};

END_VISP_NAMESPACE

#endif
