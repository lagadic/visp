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
 *
 *****************************************************************************/

/*!
  \file vpMeEllipse.h
  \brief Moving edges on an ellipse
*/

#ifndef vpMeEllipse_HH
#define vpMeEllipse_HH

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpMatrix.h>

#include <visp3/core/vpImagePoint.h>
#include <visp3/me/vpMeSite.h>
#include <visp3/me/vpMeTracker.h>

#include <visp3/core/vpColor.h>
#include <visp3/core/vpImage.h>

#include <list>
#include <math.h>

/*!
  \class vpMeEllipse
  \ingroup module_me

  \brief Class that tracks an ellipse moving edges.

  In this class, an ellipse is defined as the set of points \f$ (i,j) \f$ of
  the image frame (For more information about the image frame see the
  vpImagePoint documentation) that satisfy the implicit equation :

  \f[ i^2 + K_0j^2 + 2K_1ij + 2K_2i + 2K_3j + K4 = 0 \f]

  If \f$ K_0 \f$ is equal to 1 and \f$ K_1 \f$ is equal to 0 the the set of
  points \f$ (i,j) \f$ represents a circle.

  The five parameters are stored in the public attribute K.

  An ellipse is also defined thanks to three other parameter which are \f$ a
  \f$, \f$ b \f$ and \f$ e \f$. \f$ a \f$ represents the semiminor axis and
  \f$ b \f$ is the semimajor axis. Here \f$ e \f$ is the angle made by the
  major axis and the i axis of the image frame \f$ (i,j) \f$. The following
  figure shows better meaning of those parameters.

  \image html vpMeEllipse.gif
  \image latex vpMeEllipse.ps  width=10cm

  It is possible to compute the coordinates \f$ (i,j) \f$ of a point which
  belongs to the ellipse thanks to the following equations :

  \f[ i = i_c + b cos(e) cos(\alpha) - a sin(e) sin(\alpha) \f]
  \f[ j = j_c + b sin(e) cos(\alpha) + a cos(e) sin(\alpha) \f]

  Here the coordinates \f$ (i_c,j_c) \f$ are the coordinates of the ellipse
  center in the image frame and \f$ \alpha \f$ is an angle beetween \f$
  [0,2\pi] \f$ and which enables to describe all the points of the ellipse.

  \image html vpMeEllipse2.gif
  \image latex vpMeEllipse2.ps  width=10cm

  The example below available in tutorial-me-ellipse-tracker.cpp and described
  in \ref tutorial-tracking-me, section \ref tracking_me_ellipse shows how to
  use this class.

  \include tutorial-me-ellipse-tracker.cpp

*/
class VISP_EXPORT vpMeEllipse : public vpMeTracker
{
public:
  vpMeEllipse();
  vpMeEllipse(const vpMeEllipse &meellipse);
  virtual ~vpMeEllipse();

  /*!
    \return Expected number of moving edges to track along the ellipse.
   */
  int getExpectedDensity() { return (int)expecteddensity; };
  void track(const vpImage<unsigned char> &Im);

  void initTracking(const vpImage<unsigned char> &I);
  void initTracking(const vpImage<unsigned char> &I, const unsigned int n, vpImagePoint *iP);
  void initTracking(const vpImage<unsigned char> &I, const std::vector<vpImagePoint> &iP);
  void initTracking(const vpImage<unsigned char> &I, const vpImagePoint &ic, double a_p, double b_p, double e_p,
                    double low_alpha, double high_alpha);
  void display(const vpImage<unsigned char> &I, vpColor col);
  void display(const vpImage<unsigned char> &I)
  {
    vpMeTracker::display(I);
  } // Shouldn't be here since it's already in vpMeTracker
  void printParameters();

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
  /*!
    @name Deprecated functions
  */
  //@{
  void initTracking(const vpImage<unsigned char> &I, const unsigned int n, unsigned *i, unsigned *j);
//@}
#endif // VISP_BUILD_DEPRECATED_FUNCTIONS

  /*!
    Gets the 0 order moment \f$ m_{00} \f$ which represents the area of the
    ellipse.

    \return the value of \f$ m_{00} \f$.
  */
  inline double get_m00() const { return m00; }

  /*!
    Gets the 1 order raw moment \f$ m_{10} \f$ with \f$ m_{nm} = \sum_{i,j}i^n
    j^m \f$.

    \return the value of \f$ m_{10} \f$.
  */
  inline double get_m10() const { return m10; }

  /*!
    Gets the 1 order raw moment \f$ m_{01} \f$ with \f$ m_{nm} = \sum_{i,j}i^n
    j^m \f$.

    \return the value of \f$ m_{01} \f$.
  */
  inline double get_m01() const { return m01; }

  /*!
    Gets the 2 order raw moment \f$ m_{11} \f$ with \f$ m_{nm} = \sum_{i,j}i^n
    j^m \f$.

    \return the value of \f$ m_{11} \f$.
  */
  inline double get_m11() const { return m11; }

  /*!
    Gets the 2 order raw moment \f$ m_{20} \f$ with \f$ m_{nm} = \sum_{i,j}i^n
    j^m \f$.

    \return the value of \f$ m_{11} \f$.
  */
  inline double get_m20() const { return m20; }

  /*!
    Gets the 2 order raw moment \f$ m_{02} \f$ with \f$ m_{nm} = \sum_{i,j}i^n
    j^m \f$.

    \return the value of \f$ m_{11} \f$.
  */
  inline double get_m02() const { return m02; }

  /*!
    Gets the 2 order central moment \f$ \mu_{11} \f$.

    \return the value of \f$ \mu_{11} \f$.
  */
  inline double get_mu11() const { return mu11; }

  /*!
    Gets the 2 order central moment \f$ \mu_{02} \f$.

    \return the value of \f$ \mu_{02} \f$.
  */
  inline double get_mu02() const { return mu02; }

  /*!
    Gets the 2 order central moment \f$ \mu_{20} \f$.

    \return the value of \f$ \mu_{20} \f$.
  */
  inline double get_mu20() const { return mu20; }

  /*!
    Gets the center of the ellipse.
  */
  inline vpImagePoint getCenter() const { return iPc; }

  /*!
    Gets the semiminor axis of the ellipse.
  */
  inline double getA() const { return a; }

  /*!
    Gets the semimajor axis of the ellipse.
  */
  inline double getB() const { return b; }

  /*!
    Gets the angle made by the major axis and the i axis of the image frame
    \f$ (i,j) \f$
  */
  inline double getE() const { return e; }

  /*!
    Gets the equation parameters of the ellipse
  */
  void getEquationParam(double &A, double &B, double &E)
  {
    A = a;
    B = b;
    E = e;
  }

  /*!
    Gets the smallest \f$ alpha \f$ angle
  */
  inline double getSmallestAngle() { return alpha1; }

  /*!
    Gets the highest \f$ alpha \f$ angle
  */
  inline double getHighestAngle() { return alpha2; }

  /*!
    Set the new threshold for the robust estimation of the parameters of the
    ellipse equation.
    If the weight of a point is below this threshold, this one is removed from
    the list of tracked meSite.
    Value must be between 0 (never rejected) and 1 (always rejected).

    \param threshold : The new value of the threshold.
  */
  void setThresholdRobust(const double threshold)
  {
    if (threshold < 0) {
      thresholdWeight = 0;
    } else if (threshold > 1) {
      thresholdWeight = 1;
    } else {
      thresholdWeight = threshold;
    }
  }

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
public:
#else
protected:
#endif
  /*! Parameters of the ellipse to define the set of points that satisfy the
   implicit equation : \f[ i^2 + K_0j^2 + 2K_1ij + 2K_2i + 2K_3j + K4 = 0 \f]
  */
  vpColVector K;
  //! The coordinates of the ellipse center.
  vpImagePoint iPc;
  //! \f$ a \f$ is the semiminor axis of the ellipse.
  double a;
  //! \f$ b \f$ is the semimajor axis of the ellipse.
  double b;
  //! \f$ e \f$ is the angle made by the major axis and the i axis of the
  //! image frame \f$ (i,j) \f$.
  double e;

protected:
  //! The coordinates of the point corresponding to the smallest \f$ alpha \f$
  //! angle. More things about the \f$ alpha \f$ are given at the beginning of
  //! the class description.
  vpImagePoint iP1;
  //! The coordinates of the point corresponding to the highest \f$ alpha \f$
  //! angle. More things about the \f$ alpha \f$ are given at the beginning of
  //! the class description.
  vpImagePoint iP2;
  //! The smallest \f$ alpha \f$ angle.
  double alpha1;
  //! The highest \f$ alpha \f$ angle.
  double alpha2;
  //! Value of cos(e).
  double ce;
  //! Value of sin(e).
  double se;
  //! Stores the value of the \f$ alpha \f$ angle for each vpMeSite.
  std::list<double> angle;
  //! Surface
  double m00;
  //! Second order central moments
  double mu11, mu20, mu02;
  //! First order raw moments
  double m10, m01;
  //! Second order raw moments
  double m11, m02, m20;
  //! Threshold for the robust least square.
  double thresholdWeight;
  //! Expected number of me to track along the ellipse.
  double expecteddensity;

private:
  void computeAngle(const vpImagePoint &pt1, const vpImagePoint &pt);
  void sample(const vpImage<unsigned char> &image);
  void reSample(const vpImage<unsigned char> &I);
  void leastSquare();
  void updateTheta();
  void suppressPoints();
  void seekExtremities(const vpImage<unsigned char> &I);
  void setExtremities();
  void getParameters();
  void computeMoments();

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
  /*!
    @name Deprecated functions
  */
  //@{
  void computeAngle(int ip1, int jp1, int ip2, int jp2);
  void computeAngle(int ip1, int jp1, double &alpha1, int ip2, int jp2, double &alpha2);
//@}
#endif // VISP_BUILD_DEPRECATED_FUNCTIONS

  // Static Function
public:
  static void display(const vpImage<unsigned char> &I, const vpImagePoint &center, const double &A, const double &B,
                      const double &E, const double &smallalpha, const double &highalpha,
                      const vpColor &color = vpColor::green, unsigned int thickness = 1);
  static void display(const vpImage<vpRGBa> &I, const vpImagePoint &center, const double &A, const double &B,
                      const double &E, const double &smallalpha, const double &highalpha,
                      const vpColor &color = vpColor::green, unsigned int thickness = 1);
};

#endif
