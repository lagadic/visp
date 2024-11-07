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
 */

/*!
 * \file vpMeEllipse.h
 * \brief Moving edges on an ellipse
 */

#ifndef VP_ME_ELLIPSE_H
#define VP_ME_ELLIPSE_H

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpMatrix.h>

#include <visp3/core/vpImagePoint.h>
#include <visp3/me/vpMeSite.h>
#include <visp3/me/vpMeTracker.h>

#include <visp3/core/vpColor.h>
#include <visp3/core/vpImage.h>

#include <list>
#include <math.h>

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_17)
#include <optional>
#endif

BEGIN_VISP_NAMESPACE

/*!
 * \class vpMeEllipse
 * \ingroup module_me
 *
 * \brief Class that tracks an ellipse using moving edges.
 *
 * In this class, an ellipse is defined as the set of image points \f$ (u,v) \f$
 * (for more information about the image frame see the vpImagePoint
 * documentation) that satisfy the homogeneous equation:
 *
 * \f[ K_0 u^2 + K_1 v^2 + 2K_2 u v + 2K_3 u + 2K_4 v + K5 = 0 \f]
 * with \f$K_0 = n_{02}\f$, \f$K_1 = n_{20}\f$, \f$K_2 = -n_{11}\f$, etc.
 * (see Eq. (25) of Chaumette TRO 2004 paper \cite Chaumette04a)
 *
 * If \f$ K_0 = K1 \f$ and \f$ K_2 = 0 \f$, the ellipse is a circle.
 *
 * The six parameters are stored in the private attribute K.
 *
 * An ellipse is also represented thanks to five parameters that are
 * the center \f$ (uc,vc) \f$ of the ellipse and either:
 * - three normalized moments \f$ n_{ij} \f$ of order 2,
 * - or the semi major axis \f$ A \f$, the semi minor
 *   axis \f$ B \f$ and the ellipse orientation \f$ E \in [-\pi/2;\pi/2] \f$
 *   defined by the angle between the major axis and the u axis of the image frame.
 *
 * For more details, see \cite Chaumette04a.
 * The following figure illustrates these parameters.
 *
 * \image html vpMeEllipse.gif
 *
 * The example below available in tutorial-me-ellipse-tracker.cpp and described
 * in \ref tutorial-tracking-me, section \ref tracking_me_ellipse shows how to
 * use this class.
 *
 * \include tutorial-me-ellipse-tracker.cpp
 *
*/
class VISP_EXPORT vpMeEllipse : public vpMeTracker
{
public:
  /*!
   * Basic constructor that calls the constructor of the class vpMeTracker.
   */
  vpMeEllipse();

  /*!
   * Copy constructor.
   */
  vpMeEllipse(const vpMeEllipse &me_ellipse);

  /*!
   * Destructor.
   */
  virtual ~vpMeEllipse() VP_OVERRIDE;

  /*!
   * Display the ellipse or arc of ellipse
   *
   * \warning To effectively display the ellipse a call to
   * vpDisplay::flush() is needed.
   *
   * \param I : Image in which the ellipse appears.
   * \param col : Color of the displayed ellipse.
   * \param thickness : Thickness of the drawing.
   */
  void display(const vpImage<unsigned char> &I, const vpColor &col, unsigned int thickness = 1);

  /*!
   * Gets the second order normalized centered moment \f$ n_{ij} \f$
   * as a 3-dim vector containing \f$ n_{20}, n_{11}, n_{02} \f$
   * such as \f$ n_{ij} = \mu_{ij}/m_{00} \f$
   *
   * \return The 3-dim vector containing \f$ n_{20}, n_{11}, n_{02} \f$.
   *
   * \sa getCenter(), get_ABE(), getArea()
   */
  inline vpColVector get_nij() const
  {
    vpColVector nij(3);
    const unsigned int index_0 = 0;
    const unsigned int index_1 = 1;
    const unsigned int index_2 = 2;
    nij[index_0] = m_n20;
    nij[index_1] = m_n11;
    nij[index_2] = m_n02;

    return nij;
  }

  /*!
   * Gets the ellipse parameters as a 3-dim vector containing \f$ A, B, E \f$.
   *
   * \return The 3-dim vector containing \f$ A, B, E \f$ corresponding respectively to
   * the semi major axis, the semi minor axis and the angle in rad made by the major axis
   * and the u axis of the image frame \f$ (u,v) \f$,  \f$ e \in [-\pi/2;pi/2] \f$.
   *
   * \sa getCenter(), get_nij(), getArea()
   */
  inline vpColVector get_ABE() const
  {
    vpColVector ABE(3);
    const unsigned int index_0 = 0;
    const unsigned int index_1 = 1;
    const unsigned int index_2 = 2;
    ABE[index_0] = m_a;
    ABE[index_1] = m_b;
    ABE[index_2] = m_e;

    return ABE;
  }

  /*!
   * Gets the area of the ellipse corresponding also to the zero
   * order moment of the ellipse.
   *
   * \return The ellipse area.
   *
   * \sa getCenter(), get_nij(), get_ABE()
   */
  inline double getArea() const { return m_m00; }

  /*!
   * Gets the center of the ellipse.
   *
   * \sa get_nij(), get_ABE(), getArea()
   */
  inline vpImagePoint getCenter() const { return m_iPc; }

  /*!
   * \return Expected number of moving edges to track along the ellipse.
   */
  unsigned int getExpectedDensity() const { return m_expectedDensity; }

  /*!
   * \return Number of valid edges tracked along the ellipse.
   */
  unsigned int getNumberOfGoodPoints() const { return m_numberOfGoodPoints; }

  /*!
   * Gets the first endpoint of the ellipse arc (corresponding to alpha1,
   * not alphamin) when an arc is tracked.
   *
   * \sa getSecondEndpoint()
   */
  inline vpImagePoint getFirstEndpoint() const { return m_iP1; }

  /*!
   * Gets the highest \f$ alpha \f$ angle of the moving edges tracked
   * \f$ \alpha_{max} \in [\alpha_min;\alpha_2] \f$.
   *
   * \sa getSmallestAngle()
   */
  inline double getHighestAngle() const { return m_alphamax; }

  /*!
   * Gets the second endpoint of the ellipse arc (corresponding to alpha2,
   * not alphamax) when an arc is tracked.
   *
   * \sa getFirstEndpoint()
   */
  inline vpImagePoint getSecondEndpoint() const { return m_iP2; }

  /*!
   * Gets the smallest \f$ alpha \f$ angle of the moving edges tracked
   * \f$ \alpha_{min} \in [\alpha_1;\alpha_2] \f$.
   *
   * \sa getHighestAngle()
   */
  inline double getSmallestAngle() const { return m_alphamin; }

  /*!
   * Initialize the tracking of an ellipse or an arc of an ellipse when \e trackArc is set to true.
   * Ask the user to click on five points located on the ellipse to be tracked.
   *
   * \warning The points should be selected as far as possible from each other.
   * When an arc of an ellipse is tracked, it is recommended to select the 5 points clockwise.
   *
   * \param I : Image in which the ellipse appears.
   * \param trackCircle : When true, track a circle, when false track an ellipse.
   * \param trackArc : When true track an arc of the ellipse/circle. In that case, first and
   * last points specify the extremities of the arc (clockwise).
   * When false track the complete ellipse/circle.
   */
  void initTracking(const vpImage<unsigned char> &I, bool trackCircle = false, bool trackArc = false);

  /*!
   * Initialize the tracking of an ellipse or an arc of an ellipse when \e trackArc is set to true.
   * If \b ips is set, use the contained points to initialize the ME if there are some, or initialize
   * by clicks the ME and \b ips will contained the clicked points.
   * If \b ips is not set, call the method vpMeEllipse::initTracking(const vpImage<unsigned char>&, bool, bool).
   *
   * \sa vpMeEllipse::initTracking()
   *
   * \warning The points should be selected as far as possible from each other.
   * When an arc of an ellipse is tracked, it is recommended to select the 5 points clockwise.
   *
   * \param I : Image in which the ellipse appears.
   * \param opt_ips: If set, either a vector that contains the vpImagePoint to use or will be filled with the clicked
   * points.
   * \param trackCircle : When true, track a circle, when false track an ellipse.
   * \param trackArc : When true track an arc of the ellipse/circle. In that case, first and
   * last points specify the extremities of the arc (clockwise).
   * When false track the complete ellipse/circle.
   */
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_17)
  void initTracking(const vpImage<unsigned char> &I, std::optional<std::vector<vpImagePoint>> &opt_ips, bool trackCircle = false, bool trackArc = false);
#else
  void initTracking(const vpImage<unsigned char> &I, std::vector<vpImagePoint> *opt_ips, bool trackCircle = false, bool trackArc = false);
#endif

/*!
 * Initialize the tracking of an ellipse/circle or an arc of an ellipse/circle when \e trackArc is set to true.
 * The ellipse/circle is defined thanks to a vector of image points.
 *
 * \warning It is mandatory to use at least five image points to estimate the
 * ellipse parameters while three points are needed to estimate the circle parameters.
 * \warning The image points should be selected as far as possible from each other.
 * When an arc of an ellipse/circle is tracked, it is recommended to select the 5/3 points clockwise.
 *
 * \param I : Image in which the ellipse/circle appears.
 * \param iP : A vector of image points belonging to the ellipse/circle edge used to
 * initialize the tracking.
 * \param trackCircle : When true, track a circle, when false track an ellipse.
 * \param trackArc : When true track an arc of the ellipse/circle. In that case, first and
 * last points specify the extremities of the arc (clockwise).
 * When false track the complete ellipse/circle.
 */
  void initTracking(const vpImage<unsigned char> &I, const std::vector<vpImagePoint> &iP, bool trackCircle = false,
                    bool trackArc = false);

  /*!
   * Initialize the tracking of an ellipse/circle or an arc of an ellipse/circle when arc extremities are given.
   * The ellipse/circle is defined by the vector containing the coordinates of its center and the three second order
   * centered normalized moments \f$ n_ij \f$. Without setting the arc extremities with
   * parameters \e pt1 and \e pt2, the complete ellipse/circle is considered. When extremities
   * are set, we consider an ellipse/circle arc defined clockwise from first extremity to second extremity.
   *
   * \param I : Image in which the ellipse appears.
   * \param param : Vector with the five parameters \f$(u_c, v_c, n_{20}, n_{11}, n_{02})\f$ defining the ellipse
   * (expressed in pixels).
   * \param pt1 : Image point defining the first extremity of the arc or nullptr to track a complete ellipse.
   * \param pt2 : Image point defining the second extremity of the arc or nullptr to track a complete ellipse.
   * \param trackCircle : When true enable tracking of a circle, when false the tracking of an ellipse.
   */
  void initTracking(const vpImage<unsigned char> &I, const vpColVector &param, vpImagePoint *pt1 = nullptr,
                    const vpImagePoint *pt2 = nullptr, bool trackCircle = false);

  /*!
   * Print the parameters \f$ K = {K_0, ..., K_5} \f$, the coordinates of the
   * ellipse center, the normalized moments, and the A, B, E parameters.
   */
  void printParameters() const;

  /*!
   * Set the two endpoints of the ellipse arc when an arc is tracked.
   */
  void setEndpoints(const vpImagePoint &pt1, const vpImagePoint &pt2)
  {
    m_iP1 = pt1;
    m_iP2 = pt2;
  }

  /*!
   * Set the threshold for the weights in the robust estimation of the
   * ellipse parameters.
   * If the weight of a point is below this threshold, this one is removed from
   * the list of tracked meSite.
   * Value must be between 0 (never rejected) and 1 (always rejected).
   *
   * \param threshold : The new value of the threshold.
   */
  void setThresholdRobust(double threshold)
  {
    if (threshold < 0) {
      m_thresholdWeight = 0;
    }
    else if (threshold > 1) {
      m_thresholdWeight = 1;
    }
    else {
      m_thresholdWeight = threshold;
    }
  }

  /*!
   * Track a set of MEs along an ellipse or a circle.
   * The number of valid tracked MEs is obtained from getNumberOfGoodPoints().
   */
  void track(const vpImage<unsigned char> &I);

  /*!
   * Display the ellipse or the arc of ellipse thanks to the ellipse parameters.
   *
   * \param I : The image used as background.
   * \param center : Center of the ellipse.
   * \param A : Semi major axis of the ellipse.
   * \param B : Semi minor axis of the ellipse.
   * \param E : Angle made by the major axis and the u axis of the image frame
   * \f$ (u,v) \f$ (in rad).
   * \param smallalpha : Smallest \f$ alpha \f$ angle in rad (0 for a complete ellipse).
   * \param highalpha : Highest \f$ alpha \f$ angle in rad (2 \f$ \Pi \f$ for a complete ellipse).
   * \param color : Color used to display the ellipse.
   * \param thickness : Thickness of the drawings.
   *
   * \sa vpDisplay::displayEllipse()
   */
  static void displayEllipse(const vpImage<unsigned char> &I, const vpImagePoint &center, const double &A, const double &B,
                             const double &E, const double &smallalpha, const double &highalpha,
                             const vpColor &color = vpColor::green, unsigned int thickness = 1);

  /*!
   * Display the ellipse or the arc of ellipse thanks to the ellipse parameters.
   *
   * \param I : The image used as background.
   * \param center : Center of the ellipse
   * \param A : Semi-major axis of the ellipse.
   * \param B : Semi-minor axis of the ellipse.
   * \param E : Angle made by the major axis and the u axis of the image frame
   * \f$ (u,v) \f$ (in rad)
   * \param smallalpha : Smallest \f$ alpha \f$ angle in rad  (0 for a complete ellipse)
   * \param highalpha : Highest \f$ alpha \f$ angle in rad  (\f$ 2 \Pi \f$ for a complete ellipse)
   * \param color : Color used to display th lines.
   * \param thickness : Thickness of the drawings.
   *
   * \sa vpDisplay::displayEllipse()
   */
  static void displayEllipse(const vpImage<vpRGBa> &I, const vpImagePoint &center, const double &A, const double &B,
                             const double &E, const double &smallalpha, const double &highalpha,
                             const vpColor &color = vpColor::green, unsigned int thickness = 1);

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
  // Marked deprecated since they override vpMeTracker::display(). Warning detected by mingw64
  /*!
   * \deprecated Use rather displayEllipse().
   */
  VP_DEPRECATED static void display(const vpImage<unsigned char> &I, const vpImagePoint &center, const double &A, const double &B,
                                    const double &E, const double &smallalpha, const double &highalpha,
                                    const vpColor &color = vpColor::green, unsigned int thickness = 1);

  /*!
   * \deprecated Use rather displayEllipse().
   */
  VP_DEPRECATED static void display(const vpImage<vpRGBa> &I, const vpImagePoint &center, const double &A, const double &B,
                                    const double &E, const double &smallalpha, const double &highalpha,
                                    const vpColor &color = vpColor::green, unsigned int thickness = 1);
#endif

protected:
  /*! Parameters of the ellipse satisfying the homogeneous equation :
   * \f[ K_0 u^2 + K_1 v^2 + 2K_2 uv + 2K_3u + 2K_4v + K5 = 0 \f]
   */
  vpColVector m_K;
  //! The coordinates of the ellipse center.
  vpImagePoint m_iPc;
  //! \f$ a \f$ is the semi major axis of the ellipse.
  double m_a;
  //! \f$ b \f$ is the semi minor axis of the ellipse.
  double m_b;
  /*! \f$ e \in [-\pi/2;\pi/2] \f$ is the angle made by the major axis
   * and the u axis of the image frame \f$ (u,v) \f$.
   */
  double m_e;

  /*! The coordinates of the first endpoint of the ellipse arc
   *  corresponding to angle \f$ \alpha_1 \f$
   */
  vpImagePoint m_iP1;
  /*! The coordinates of the second endpoint of the ellipse arc
   *  corresponding to angle \f$ \alpha_2 \f$
   */
  vpImagePoint m_iP2;
  /*! The angle \f$ \alpha_1 \in [-\pi;\pi] \f$ on the ellipse corresponding
   *  to the first endpoint. Its value is 0 for tracking a complete ellipse
   */
  double m_alpha1;
  /*! The angle \f$ \alpha_2 \in [\alpha_1;\alpha_1+2\pi]\f$ on the ellipse
   *  corresponding to the second endpoint. Its value is \f$ 2 \pi \f$ for
   *  tracking a complete ellipse
   */
  double m_alpha2;
  //! Value of cos(e).
  double m_ce;
  //! Value of sin(e).
  double m_se;
  //! Stores the value in increasing order of the \f$ alpha \f$ angle on the ellipse for each vpMeSite.
  std::list<double> m_angleList;
  //! Ellipse area
  double m_m00;

  //! Threshold on the weights for the robust least square.
  double m_thresholdWeight;

  /*! The smallest angle \f$ \alpha_{min} \in [\alpha_1;\alpha_2]\f$
   *  of the current moving edge list
   */
  double m_alphamin;
  /*! The highest angle \f$ \alpha_{max} \in [\alpha_{min};\alpha_2]\f$
   *  of the current moving edge list
   */
  double m_alphamax;
  //! Value of u coordinate of iPc
  double m_uc;
  //! Value of v coordinate of iPc
  double m_vc;
  //! Second order centered and normalized moments \f$ n_{20} \f$
  double m_n20;
  //! Second order centered and normalized moments \f$ n_{11} \f$
  double m_n11;
  //! Second order centered and normalized moments \f$ n_{02} \f$
  double m_n02;
  //! Expected number of points to track along the ellipse.
  unsigned int m_expectedDensity;
  //! Number of correct points tracked along the ellipse.
  unsigned int m_numberOfGoodPoints;
  //! Track a circle (true) or an ellipse (false).
  bool m_trackCircle;
  //! Track an arc of ellipse/circle (true) or a complete one (false).
  bool m_trackArc;
  //! Epsilon value used to check if arc angles are the same
  double m_arcEpsilon;

  /*!
   * Computes the length of the semi major axis \f$ a \f$, the length of the
   * semi minor axis \f$ b \f$, and \f$ e \f$ that is the angle
   * made by the major axis and the u axis of the image frame \f$ (u,v) \f$.
   * They are computed from the normalized moments $ \f$ n_{ij} \f$.
   */
  void computeAbeFromNij();

  /*!
   * Compute the angle of a point on the ellipse wrt the ellipse major axis.
   * \param pt : Image point on the ellipse.
   * \return The computed angle.
   */
  double computeAngleOnEllipse(const vpImagePoint &pt) const;

  /*!
   * Computes the parameters \f$ K = {K_0, ...,  K_5} \f$ from the center of
   * the ellipse and the normalized moments \f$ n_{ij} \f$. The parameters
   * \f$ K \f$ are such that \f$ K0 = n02, K1 = n20 \f$, etc. as in Eq (25)
   * of Chaumette 2004 TRO paper.
   */
  void computeKiFromNij();

  /*!
   * Computes the normalized moments \f$ n_{ij} \f$ from the \f$ A, B, E \f$
   * parameters as in Eq (24) of Chaumette 2004 TRO paper after simplifications
   * to deal with the case cos(e) = 0.0
   */
  void computeNijFromAbe();

  /*!
   * Compute the coordinates of a point on an ellipse from its angle with respect
   * to the main orientation of the ellipse.
   *
   * \param angle : Angle on the ellipse with respect to its major axis.
   * \param iP : Image point on the ellipse.
   */
  void computePointOnEllipse(const double angle, vpImagePoint &iP);

  /*!
   * Computes the \f$ \theta \f$ angle that represents the angle between the
   * tangent to the curve and the u axis. This angle is used for tracking the
   * vpMeSite.
   *
   * \param iP : The point belonging to the ellipse where the angle is computed.
   */
  double computeTheta(const vpImagePoint &iP) const;

  /*!
   * Computes the \f$ \theta \f$ angle that represents the angle between the
   * tangent to the curve and the u axis. This angle is used for tracking the
   * vpMeSite.
   *
   * \param u,v : The point belonging to the ellipse where the angle is computed.
   */
  double computeTheta(double u, double v) const;

  /*!
   * Computes the coordinates of the ellipse center, the normalized
   * moments \f$ n_{ij} \f$, the length of the semi major axis \f$ a \f$, the
   * length of the semi minor axis \f$ b \f$, and \f$ e \f$ that is the angle
   * made by the major axis and the u axis of the image frame \f$ (u,v) \f$.
   *
   * All those computations are made from the parameters \f$ K ={K_0, ..., K_5} \f$
   * so that \f$ K_0 u^2 + K1 v^2 + 2 K2 u v + 2 K3 u + 2 K4 v + K5 = 0 \f$.
   */
  void getParameters();

  /*!
   * Least squares method to compute the circle/ ellipse to which the points belong.
   *
   * \param I : Image in which the circle/ellipse appears (useful just to get
   * its number of rows and columns...
   * \param iP : A vector of points belonging to the circle/ellipse.
   */
  void leastSquare(const vpImage<unsigned char> &I, const std::vector<vpImagePoint> &iP);

  /*!
   * Robust least squares method to compute the ellipse to which the vpMeSite
   * belong. Manage also the lists of vpMeSite and corresponding angles,
   * and update the expected density of points.
   *
   * \param I : Image where tracking is done (useful just to get its number
   * of rows and columns...
   */
  unsigned int leastSquareRobust(const vpImage<unsigned char> &I);

  /*!
   * Robust least squares method to compute the ellipse to which the vpMeSite
   * belong, when we track a circle. Manage also the lists of vpMeSite and corresponding angles,
   * and update the expected density of points.
   *
   * \param um Half width of the image.
   * \param vm Half height of the image.
   * \param k Count the number of tracked MEs.
   * \param w Weights computed by robust estimation.
   */
  void leastSquareRobustCircle(const double &um, const double &vm, unsigned int &k, vpColVector &w);

  /*!
   * Robust least squares method to compute the ellipse to which the vpMeSite
   * belong, when we track an ellipse. Manage also the lists of vpMeSite and corresponding angles,
   * and update the expected density of points.
   *
   * \param um Half width of the image.
   * \param vm Half height of the image.
   * \param k Count the number of tracked MEs.
   * \param w Weights computed by robust estimation.
   */
  void leastSquareRobustEllipse(const double &um, const double &vm, unsigned int &k, vpColVector &w);

  /*!
   * Seek along the ellipse or arc of ellipse its two extremities to try
   * recovering lost points. Try also to complete the parts with no tracked points.
   *
   * \param I : Image in which the ellipse appears.
   *
   * \return The function returns the number of points added to the list.
   *
   * \exception vpTrackingException::initializationError : Moving edges not
   * initialized.
   */
  unsigned int plugHoles(const vpImage<unsigned char> &I);

  /*!
   * Construct a list of vpMeSite moving edges at a particular sampling
   * step between the two extremities. The two extremities are defined by
   * the points with the smallest and the biggest \f$ alpha \f$ angle.
   *
   * \param I : Image in which the ellipse appears.
   * \param doNotTrack : If true, moving-edges are not tracked.
   *
   * \exception vpTrackingException::initializationError : Moving edges not
   * initialized.
   */
  virtual void sample(const vpImage<unsigned char> &I, bool doNotTrack = false) VP_OVERRIDE;

  /*!
   * Compute the \f$ theta \f$ angle for each vpMeSite.
   *
   * \note The \f$ theta \f$ angle is useful during the tracking part.
   */
  void updateTheta();
};

END_VISP_NAMESPACE

#endif
