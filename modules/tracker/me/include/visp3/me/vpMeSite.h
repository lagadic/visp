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
 * Moving edges.
 *
 * Authors:
 * Eric Marchand
 * Andrew Comport
 * Aurelien Yol
 *
 *****************************************************************************/

/*!
        \file vpMeSite.h
        \brief Moving edges
*/

#ifndef vpMeSite_H
#define vpMeSite_H

#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/me/vpMe.h>

/*!
  \class vpMeSite
  \ingroup module_me

  \brief Performs search in a given direction(normal) for a given
   distance(pixels) for a given 'site'. Gives the most likely site
   given the probablility from an ME mask

  - Bug fix: rewrote application of masks to use the temporal
    information instead of applying both temporal masks to the same
    image. ie: spatial -> spatio/temporal

  - Added new tracking function to choose the most similar edge
    amongst all edges found.

  - sample step.
 */
class VISP_EXPORT vpMeSite
{
public:
  typedef enum { NONE, RANGE, RESULT, RANGE_RESULT } vpMeSiteDisplayType;

  typedef enum {
    NO_SUPPRESSION = 0,
    CONSTRAST = 1,
    THRESHOLD = 2,
    M_ESTIMATOR = 3,
    TOO_NEAR = 4,
    UNKNOW = 5
  } vpMeSiteState;

public:
  int i, j;
  int i_1, j_1;
  double ifloat, jfloat;
  unsigned char v;
  int mask_sign;
  // Angle of tangent at site
  double alpha;
  // Convolution of Site in previous image
  double convlt;
  // Convolution of Site in previous image
  double normGradient;
  // Uncertainty of point given as a probability between 0 and 1
  double weight;

private:
  vpMeSiteDisplayType selectDisplay;
  vpMeSiteState state;

public:
  void init();
  void init(double ip, double jp, double alphap);
  void init(double ip, double jp, double alphap, double convltp);
  void init(double ip, double jp, double alphap, double convltp, int sign);

  vpMeSite();
  vpMeSite(double ip, double jp);
  vpMeSite(const vpMeSite &mesite);
  virtual ~vpMeSite(){};

  void display(const vpImage<unsigned char> &I);

  double convolution(const vpImage<unsigned char> &ima, const vpMe *me);

  vpMeSite *getQueryList(const vpImage<unsigned char> &I, const int range);

  void track(const vpImage<unsigned char> &im, const vpMe *me, const bool test_contraste = true);

  /*!
    Set the angle of tangent at site

    \param a : new value of alpha
  */
  void setAlpha(const double &a) { alpha = a; }

  /*!
    Get the angle of tangent at site

    \return value of alpha
  */
  inline double getAlpha() const { return alpha; }

  void setDisplay(vpMeSiteDisplayType select) { selectDisplay = select; }

  /*!
    Get the i coordinate (integer)

    \return value of i
  */
  inline int get_i() const { return i; }

  /*!
    Get the j coordinate (f)

    \return value of j
  */
  inline int get_j() const { return j; }

  /*!
    Get the i coordinate (double)

    \return value of i
  */
  inline double get_ifloat() const { return ifloat; }

  /*!
    Get the j coordinate (double)

    \return value of j
  */
  inline double get_jfloat() const { return jfloat; }

  /*!
    Set the state of the site

    \param flag : flag corresponding to vpMeSiteState

    \sa vpMeSiteState
  */
  void setState(const vpMeSiteState &flag)
  {
    state = flag;

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
    suppress = (int)flag;
#endif
  }

  /*!
    Get the state of the site

    \return flag corresponding to vpMeSiteState
  */
  inline vpMeSiteState getState() const { return state; }

  /*!
    Set the weight of the site

    \param w : new value of weight
  */
  void setWeight(const double &w) { weight = w; }

  /*!
    Get the weight of the site

    \return value of weight
  */
  inline double getWeight() const { return weight; }

  // Operators
  vpMeSite &operator=(const vpMeSite &m);
  int operator!=(const vpMeSite &m);

  friend VISP_EXPORT std::ostream &operator<<(std::ostream &os, vpMeSite &vpMeS);

  // Static functions
  /*!
    Compute the distance \f$ |S1 - S2| = \sqrt{(i_1-i_2)^2+(j_1-j_2)^2} \f$

    \param S1 : First site
    \param S2 : Second site

    \return the distance between the two sites.
  */
  static double distance(const vpMeSite &S1, const vpMeSite &S2)
  {
    return (sqrt(vpMath::sqr(S1.ifloat - S2.ifloat) + vpMath::sqr(S1.jfloat - S2.jfloat)));
  }

  /*!
    Compute the distance \f$ |S1 - S2| = (i_1-i_2)^2+(j_1-j_2)^2 \f$

    \param S1 : First site
    \param S2 : Second site

    \return the distance between the two sites.
  */
  static double sqrDistance(const vpMeSite &S1, const vpMeSite &S2)
  {
    return (vpMath::sqr(S1.ifloat - S2.ifloat) + vpMath::sqr(S1.jfloat - S2.jfloat));
  }

  static void display(const vpImage<unsigned char> &I, const double &i, const double &j,
                      const vpMeSiteState &state = NO_SUPPRESSION);
  static void display(const vpImage<vpRGBa> &I, const double &i, const double &j,
                      const vpMeSiteState &state = NO_SUPPRESSION);

// Deprecated
#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
public:
  //! Flag to indicate whether point is rejected or not
  //! 1 = contrast, 2 = threshold, 3 = M-estimator, 0 = nosupp
  int suppress;

  vp_deprecated void getSign(const vpImage<unsigned char> &I, const int range);
#endif
};

#endif
