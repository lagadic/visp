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

  \brief Class that tracks an ellipse using moving edges.

  In this class, an ellipse is defined as the set of image points \f$ (u,v) \f$
  (for more information about the image frame see the vpImagePoint
  documentation) that satisfy the homogeneous equation:

  \f[ K_0 u^2 + K_1 v^2 + 2K_2  u v + 2K_3 u + 2K_4 v + K5 = 0 \f]
  with \f$K_0 = n_{02}\f$, \f$K_1 = n_{20}\f$, \f$K_2 = -n_{11}\f$, etc.
  (see Eq. (25) of Chaumette TRO 2004 paper \cite Chaumette04a)

  If \f$ K_0 = K1 \f$ and \f$ K_2 = 0 \f$, the ellipse is a circle.

  The six parameters are stored in the private attribute K.

  An ellipse is also represented thanks to five parameters that are
  the center \f$ (uc,vc) \f$ of the ellipse and either:
  - three normalized moments \f$ n_{ij} \f$ of order 2,
  - or the semimajor axis \f$ A \f$, the semiminor
  axis \f$ B \f$ and the ellipse orientation \f$ E \in [-\pi/2;\pi/2] \f$
  defined by the angle between the major axis and the u axis of the image
  frame.
  For more details, see \cite Chaumette04a.
  The following figure illustrates these parameters.

  \image html vpMeEllipse.gif

  The example below available in tutorial-me-ellipse-tracker.cpp and described
  in \ref tutorial-tracking-me, section \ref tracking_me_ellipse shows how to
  use this class.

  \include tutorial-me-ellipse-tracker.cpp

*/
class VISP_EXPORT vpMeEllipse : public vpMeTracker
{
public:
  vpMeEllipse();
  vpMeEllipse(const vpMeEllipse &me_ellipse);
  virtual ~vpMeEllipse();

  void display(const vpImage<unsigned char> &I, vpColor col);

  /*!
    Gets the second order normalized centered moment \f$ n_{ij} \f$
    as a 3-dim vector containing \f$ n_{20}, n_{11}, n_{02} \f$
    such as \f$ n_{ij}  = \mu_{ij}/m_{00} \f$

    \return The 3-dim vector containing \f$ n_{20}, n_{11}, n_{02} \f$.

    \sa getCenter(), get_ABE(), getArea()
  */
  inline vpColVector get_nij() const
  {
    vpColVector nij(3);
    nij[0] = m_n20;
    nij[1] = m_n11;
    nij[2] = m_n02;

    return nij;
  }

  /*!
    Gets the ellipse parameters as a 3-dim vector containing \f$ A, B, E \f$.

    \return The 3-dim vector containing \f$ A, B, E \f$ corresponding respectively to
    the semimajor axis, the semiminor axis and the angle in rad made by the major axis
    and the u axis of the image frame \f$ (u,v) \f$,  \f$ e \in [-\pi/2;pi/2] \f$.

    \sa getCenter(), get_nij(), getArea()
  */
  inline vpColVector get_ABE() const
  {
    vpColVector ABE(3);
    ABE[0] = a;
    ABE[1] = b;
    ABE[2] = e;

    return ABE;
  }

  /*!
    Gets the area of the ellipse corresponding also to the zero
    order moment of the ellipse.

    \return The ellipse area.

    \sa getCenter(), get_nij(), get_ABE()
  */
  inline double getArea() const { return m00; }

  /*!
    Gets the center of the ellipse.

    \sa get_nij(), get_ABE(), getArea()
  */
  inline vpImagePoint getCenter() const { return iPc; }

  /*!
    \return Expected number of moving edges to track along the ellipse.
   */
  unsigned int getExpectedDensity() const { return m_expectedDensity; }

  /*!
    Gets the first endpoint of the ellipse arc (corresponding to alpha1,
    not alphamin) when an arc is tracked.

    \sa getSecondEndpoint()
  */
  inline vpImagePoint getFirstEndpoint() const { return iP1; }

  /*!
    Gets the highest \f$ alpha \f$ angle of the moving edges tracked
    \f$ \alpha_{max} \in [\alpha_min;\alpha_2] \f$.

    \sa getSmallestAngle()
  */
  inline double getHighestAngle() const { return m_alphamax; }

  /*!
    Gets the second endpoint of the ellipse arc (corresponding to alpha2,
    not alphamax) when an arc is tracked.

    \sa getFirstEndpoint()
  */
  inline vpImagePoint getSecondEndpoint() const { return iP2; }

  /*!
    Gets the smallest \f$ alpha \f$ angle of the moving edges tracked
    \f$ \alpha_{min} \in [\alpha_1;\alpha_2] \f$.

    \sa getHighestAngle()
  */
  inline double getSmallestAngle() const { return m_alphamin; }

  void initTracking(const vpImage<unsigned char> &I, bool trackArc = false);
  void initTracking(const vpImage<unsigned char> &I, const std::vector<vpImagePoint> &iP, bool trackArc = false);
  void initTracking(const vpImage<unsigned char> &I, const vpColVector &param, vpImagePoint *pt1 = NULL, const vpImagePoint *pt2 = NULL);
  void printParameters() const;

  /*!
    Set the two endpoints of the ellipse arc when an arc is tracked.
  */
  void setEndpoints(const vpImagePoint &pt1, const vpImagePoint &pt2)
  {
    iP1 = pt1;
    iP2 = pt2;
  }

  /*!
    Set the threshold for the weights in the robust estimation of the
    ellipse parameters.
    If the weight of a point is below this threshold, this one is removed from
    the list of tracked meSite.
    Value must be between 0 (never rejected) and 1 (always rejected).

    \param threshold : The new value of the threshold.
  */
  void setThresholdRobust(double threshold)
  {
    if (threshold < 0) {
      thresholdWeight = 0;
    } else if (threshold > 1) {
      thresholdWeight = 1;
    } else {
      thresholdWeight = threshold;
    }
  }

  void track(const vpImage<unsigned char> &I);

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
  /*!
    @name Deprecated functions
  */
  //@{
  /*!
    \deprecated You should rather use get_ABE().
    Gets the semimajor axis of the ellipse.

    \sa get_ABE()
  */
  vp_deprecated inline double getA() const { return a; }

  /*!
    \deprecated You should rather use get_ABE().
    Gets the semiminor axis of the ellipse.

    \sa get_ABE()
  */
  vp_deprecated inline double getB() const { return b; }

  /*!
    \deprecated You should rather use get_ABE().
    Gets the angle made by the major axis and the u axis of the image frame
    \f$ (u,v) \f$,  \f$ e \in [-\pi/2;pi/2] \f$.

    \sa get_ABE()
  */
  vp_deprecated inline double getE() const { return e; }

  /*!
    \deprecated Call rather getArea().

    Gets the zero order moment \f$ m_{00} \f$ which represents the area of the
    ellipse.

    \return The value of \f$ m_{00} \f$.

    \sa getArea()
  */
  vp_deprecated inline double get_m00() const { return m00; }

  /*!
    \deprecated Gets the first order raw moment \f$ m_{10} \f$ with \f$ m_{nm} = \sum_{i,j}i^n
    j^m \f$.

    \return the value of \f$ m_{10} \f$.
  */
  vp_deprecated inline double get_m10() const { return m10; }

  /*!
    \deprecated Gets the first order raw moment \f$ m_{01} \f$ with \f$ m_{nm} = \sum_{i,j}i^n
    j^m \f$.

    \return the value of \f$ m_{01} \f$.
  */
  vp_deprecated inline double get_m01() const { return m01; }

  /*!
    \deprecated Gets the second order raw moment \f$ m_{11} \f$ with \f$ m_{nm} = \sum_{i,j}i^n
    j^m \f$.

    \return the value of \f$ m_{11} \f$.
  */
  vp_deprecated inline double get_m11() const { return m11; }

  /*!
    \deprecated Gets the second order raw moment \f$ m_{20} \f$ with \f$ m_{nm} = \sum_{i,j}i^n
    j^m \f$.

    \return the value of \f$ m_{11} \f$.
  */
  vp_deprecated inline double get_m20() const { return m20; }

  /*!
    \deprecated Gets the second order raw moment \f$ m_{02} \f$ with \f$ m_{nm} = \sum_{i,j}i^n
    j^m \f$.

    \return the value of \f$ m_{11} \f$.
  */
  vp_deprecated inline double get_m02() const { return m02; }

  /*!
    \deprecated Gets the second order centered moment \f$ \mu_{11} \f$.

    \return the value of \f$ \mu_{11} \f$.
  */
  vp_deprecated inline double get_mu11() const { return mu11; }

  /*!
    \deprecated Gets the second order centered moment \f$ \mu_{02} \f$.

    \return the value of \f$ \mu_{02} \f$.
  */
  vp_deprecated inline double get_mu02() const { return mu02; }

  /*!
    \deprecated Gets the second order centered moment \f$ \mu_{20} \f$.

    \return the value of \f$ \mu_{20} \f$.
  */
  vp_deprecated inline double get_mu20() const { return mu20; }
  /*!
    \deprecated You should rather use get_ABE().
    Gets the equation parameters of the ellipse.
  */
  vp_deprecated void getEquationParam(double &A, double &B, double &E)
  {
    A = a;
    B = b;
    E = e;
  }
  vp_deprecated void initTracking(const vpImage<unsigned char> &I, const vpImagePoint &center_p, double a_p, double b_p, double e_p,
                                  double low_alpha, double high_alpha);
  vp_deprecated void initTracking(const vpImage<unsigned char> &I, unsigned int n, vpImagePoint *iP);
  vp_deprecated void initTracking(const vpImage<unsigned char> &I, unsigned int n, unsigned *i, unsigned *j);
  //@}
#endif // VISP_BUILD_DEPRECATED_FUNCTIONS

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
public:
#else
protected:
#endif
  /*! Parameters of the ellipse satisfying the homogeneous equation :
  \f[ K_0 u^2 + K_1 v^2 + 2K_2 uv + 2K_3u + 2K_4v + K5 = 0 \f]
  */
  vpColVector K;
  //! The coordinates of the ellipse center.
  vpImagePoint iPc;
  //! \f$ a \f$ is the semimajor axis of the ellipse.
  double a;
  //! \f$ b \f$ is the semiminor axis of the ellipse.
  double b;
  /*! \f$ e \in [-\pi/2;\pi/2] \f$ is the angle made by the major axis
    and the u axis of the image frame \f$ (u,v) \f$.
  */
  double e;

protected:
  /*! The coordinates of the first endpoint of the ellipse arc
      corresponding to angle \f$ \alpha_1 \f$
  */
  vpImagePoint iP1;
  /*! The coordinates of the second endpoint of the ellipse arc
      corresponding to angle \f$ \alpha_2 \f$
  */
  vpImagePoint iP2;
  /*! The angle \f$ \alpha_1 \in [-\pi;\pi] \f$ on the ellipse corresponding
      to the first endpoint. Its value is 0 for tracking a complete ellipse
  */
  double alpha1;
  /*! The angle \f$ \alpha_2 \in [\alpha_1;\alpha_1+2\pi]\f$ on the ellipse
      corresponding to the second endpoint. Its value is \f$ 2 \pi \f$ for
      tracking a complete ellipse
   */
  double alpha2;
  //! Value of cos(e).
  double ce;
  //! Value of sin(e).
  double se;
  //! Stores the value in increasing order of the \f$ alpha \f$ angle on the ellipse for each vpMeSite .
  std::list<double> angle;
  //! Ellipse area
  double m00;
#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
  //! Second order centered moments
  double mu11, mu20, mu02;
  //! First order raw moments
  double m10, m01;
  //! Second order raw moments
  double m11, m02, m20;
#endif

  //! Threshold on the weights for the robust least square.
  double thresholdWeight;

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
  //! Expected number of points to track along the ellipse.
  double expecteddensity;
#endif

  /*! The smallest angle \f$ \alpha_{min} \in [\alpha_1;\alpha_2]\f$
      of the current moving edge list
   */
  double m_alphamin;
  /*! The highest angle \f$ \alpha_{max} \in [\alpha_{min};\alpha_2]\f$
      of the current moving edge list
   */
  double m_alphamax;
  //! Value of u coordinate of iPc
  double m_uc;
  //! Value of v coordinate of iPc
  double m_vc;
  //! Second order centered and normalized moments
  double m_n20, m_n11, m_n02;
  //! Expected number of points to track along the ellipse.
  unsigned int m_expectedDensity;
  //! Number of correct points tracked along the ellipse.
  unsigned int m_numberOfGoodPoints;
  //! Track an arc of ellipse (true) or a complete one (false).
  bool m_trackArc;
  //! Epsilon value used to check if arc angles are the same
  double m_arcEpsilon;

protected:
  void computeAbeFromNij();
  double computeAngleOnEllipse(const vpImagePoint &pt) const;
  void computeKiFromNij();
  void computeNijFromAbe();
  void computePointOnEllipse(const double ang, vpImagePoint &iP);
  double computeTheta(const vpImagePoint &iP) const;
  double computeTheta(double u, double v) const;
  void getParameters();
  void leastSquare(const vpImage<unsigned char> &I, const std::vector<vpImagePoint> &iP);  // FC : new
  void leastSquareRobust(const vpImage<unsigned char> &I);
  unsigned int plugHoles(const vpImage<unsigned char> &I);
  virtual void sample(const vpImage<unsigned char> &I, bool doNotTrack=false);
  void updateTheta();

private:
#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
  void computeMoments();
#endif

  // Static Function
public:
  static void display(const vpImage<unsigned char> &I, const vpImagePoint &center,
                      const double &A, const double &B, const double &E,
                      const double &smallalpha, const double &highalpha,
                      const vpColor &color = vpColor::green, unsigned int thickness = 1);
  static void display(const vpImage<vpRGBa> &I, const vpImagePoint &center,
                      const double &A, const double &B, const double &E,
                      const double &smallalpha, const double &highalpha,
                      const vpColor &color = vpColor::green, unsigned int thickness = 1);
};

#endif
