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
 * \file vpMeLine.h
 * \brief Moving edges on a line
*/

#ifndef VP_ME_LINE_H
#define VP_ME_LINE_H

#include <visp3/core/vpMath.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/me/vpMeTracker.h>

#include <iostream>
#include <math.h>

BEGIN_VISP_NAMESPACE

/*!
 * \class vpMeLine
 *
 * \ingroup module_me
 *
 * \brief Class that tracks in an image a line moving edges.
 *
 * In this class the line is defined by its equation in the \f$ (i,j) =
 * (line,column) \f$ image plane. Two kinds of parametrization are available to
 * describe a 2D line. The first one corresponds to the following
 * equation
 *
 * \f[ ai + bj + c = 0 \f]
 *
 * where \f$ i \f$ and \f$ j \f$ are the coordinates of the points
 * belonging to the line. The line features are \f$ (a, b, c) \f$.
 *
 * The second way to write the line equation is to consider polar coordinates
 * \f[ i \; cos(\theta) + j \; sin(\theta) - \rho = 0 \f]
 *
 * where \f$ i \f$ and \f$ j \f$ are still the coordinates of the
 * points belonging to the line. But now the line features are \f$
 * (\rho, \theta) \f$. The computation of \f$ \rho \f$ and \f$ \theta
 * \f$ is easy thanks to \f$ (a, b, c) \f$.
 *
 * \f[ \theta = arctan(b/a) \f]
 * \f[ \rho = -c/\sqrt{a^2+b^2} \f]
 *
 * The value of \f$ \theta \f$ is between \f$ 0 \f$ and \f$ 2\pi
 * \f$. And the value of \f$ \rho \f$ can be positive or negative. The
 * conventions to find the right values of the two features are
 * illustrated in the following pictures.
 *
 * \image html vpMeLine.gif
 * \image latex vpMeLine.ps * width=10cm
 *
 * The angle \f$\theta\f$ is computed thanks to the direction of the
 * arrow. The arrow points to the side of the line which is darker.
 *
 * The example below available in tutorial-me-line-tracker.cpp and described
 * in \ref tutorial-tracking-me shows how to use this class.
 *
 * \include tutorial-me-line-tracker.cpp
 *
 * The code below shows how to use this class.
 * \code
 * #include <visp3/core/vpConfig.h>
 * #include <visp3/core/vpImage.h>
 * #include <visp3/core/vpImagePoint.h>
 * #include <visp3/me/vpMeLine.h>
 *
 * #ifdef ENABLE_VISP_NAMESPACE
 * using namespace VISP_NAMESPACE_NAME;
 * #endif
 *
 * int main()
 * {
 *   vpImage<unsigned char> I(240, 320);
 *
 *   // Fill the image with a black rectangle
 *   I = 0u;
 *   for (int i = 100; i < 180; i ++) {
 *     for (int j = 120; j < 250; j ++) {
 *       I[i][j] = 255;
 *     }
 *   }
 *
 *   // Set the moving-edges tracker parameters
 *   vpMe me;
 *   me.setRange(25);
 *   me.setLikelihoodThresholdType(vpMe::NORMALIZED_THRESHOLD);
 *   me.setThreshold(20);
 *   me.setSampleStep(10);
 *
 *   // Initialize the moving-edges line tracker parameters
 *   vpMeLine line;
 *   line.setMe(&me);
 *
 *   // Initialize the location of the vertical line to track
 *   vpImagePoint ip1, ip2; // Two points belonging to the line to track
 *   ip1.set_i( 120 );
 *   ip1.set_j( 119 );
 *   ip2.set_i( 170 );
 *   ip2.set_j( 122 );
 *
 *   line.initTracking(I, ip1, ip2);
 *
 *   while ( 1 )
 *   {
 *     // ... Here the code to read or grab the next image.
 *
 *     // Track the line.
 *     line.track(I);
 *   }
 *   return 0;
 * }
 * \endcode
 *
 * \note It is possible to display the line as an overlay. For that you
 * must use the display function of the class vpMeLine.
*/
class VISP_EXPORT vpMeLine : public vpMeTracker
{
public:
  /*!
   * Basic constructor that calls the constructor of the class vpMeTracker.
   */
  vpMeLine();

  /*!
   * Copy constructor.
   */
  vpMeLine(const vpMeLine &meline);

  /*!
   * Destructor.
   */
  virtual ~vpMeLine() VP_OVERRIDE;

  /*!
   * Display me line.
   *
   * \warning To effectively display the line a call to vpDisplay::flush() is needed.
   *
   * \param I : Image in which the line appears.
   * \param color : Color of the displayed line. Note that a moving edge
   * that is considered as an outlier is displayed in green.
   * \param thickness : Drawings thickness.
   */
  void display(const vpImage<unsigned char> &I, const vpColor &color, unsigned int thickness = 1);

  /*!
   * Display me line.
   *
   * \warning To effectively display the line a call to vpDisplay::flush() is needed.
   *
   * \param I : Image in which the line appears.
   * \param color : Color of the displayed line. Note that a moving edge
   * that is considered as an outlier is displayed in green.
   * \param thickness : Drawings thickness.
   */
  void display(const vpImage<vpRGBa> &I, const vpColor &color, unsigned int thickness = 1);

  /*!
   * Gets the parameters a,b,c of the line with equation a*x + b*y + c = 0 as a
   * 3-dim vector.
   *
   * \return 3-dim vector containing a, b and c line parameters.
   */
  vpColVector get_ABC() const
  {
    vpColVector abc(3);
    abc[0] = m_a;
    abc[1] = m_b;
    abc[2] = m_c;
    return abc;
  }

  /*!
   * Get the extremities of the line. These two points strictly belong to
   * the line. They are the projection of m_Pext[2] on the line
   *
   * \param ip1 : Coordinates of the first extremity.
   * \param ip2 : Coordinates of the second extremity.
   */
  void getExtremities(vpImagePoint &ip1, vpImagePoint &ip2) const;

  /*!
   * Returns the value of \f$\rho\f$, the distance between the origin and the point on the line
   * that belongs to the normal to the line passing through the origin.
   *
   * Depending on the convention described at the beginning of this
   * class, \f$\rho\f$ is signed.
   *
   * \sa getTheta(), getRhoTheta()
   */
  inline double getRho() const
  {
    return m_rho;
  }

  /*!
   * Returns the value of \f$\rho, \theta\f$ as a 2-dim vector.
   * - \f$\rho\f$ is the distance between the origin and the point on the line
   *   that belongs to the normal to the line passing through the origin.
   * - \f$\theta\f$ is the angle in radian between the vertical axis and the
   *   normal to the line passing through the origin.
   *
   * Depending on the convention described at the beginning of this
   * class, \f$\rho\f$ is signed.
   *
   * \sa getRho(), getTheta()
   */
  inline vpColVector getRhoTheta() const
  {
    vpColVector rho_theta(2);
    rho_theta[0] = m_rho;
    rho_theta[1] = m_theta;
    return rho_theta;
  }

  /*!
   * Returns the value of the angle \f$\theta\f$.
   *
   * \sa getRho(), getRhoTheta()
   */
  inline double getTheta() const
  {
    return m_theta;
  }

  /*!
   * Initialization of the tracking. Ask the user to click on two points
   * from the line to track.
   *
   * \param I : Image in which the line appears.
   */
  void initTracking(const vpImage<unsigned char> &I);

  /*!
   * Initialization of the tracking. The line is defined thanks to the
   * coordinates of two points.
   *
   * \param I : Image in which the line appears.
   * \param ip1 : Coordinates of the first point.
   * \param ip2 : Coordinates of the second point.
   */
  void initTracking(const vpImage<unsigned char> &I, const vpImagePoint &ip1, const vpImagePoint &ip2);

  /*!
   * This method allows to disable/enable the calculation of the sign of the rho attribute
   * from the intensity of the central point of the line. When enabled, it allows to distinguish
   * between a black/white edge and a white/black edge, but it can cause problems
   * for example during a visual-servo, when this point can be occluded.
   *
   * \param useIntensityForRho : new value of the flag.
   */
  inline void setRhoSignFromIntensity(bool useIntensityForRho)
  {
    m_useIntensityForRho = useIntensityForRho;
  }

  /*!
   * Track the line in the image I.
   *
   * \param I : Image in which the line appears.
   */
  void track(const vpImage<unsigned char> &I);

  /*!
   * Display of a moving line thanks to its equation parameters and its
   * extremities.
   *
   * \param I : The image used as background.
   * \param PExt1 : First extremity
   * \param PExt2 : Second extremity
   * \param A : Parameter a of the line equation a*i + b*j + c = 0
   * \param B : Parameter b of the line equation a*i + b*j + c = 0
   * \param C : Parameter c of the line equation a*i + b*j + c = 0
   * \param color : Color used to display the line.
   * \param thickness : Thickness of the line.
   */
  static void displayLine(const vpImage<unsigned char> &I, const vpMeSite &PExt1, const vpMeSite &PExt2, const double &A,
                          const double &B, const double &C, const vpColor &color = vpColor::green,
                          unsigned int thickness = 1);

  /*!
   * Display of a moving line thanks to its equation parameters and its
   * extremities.
   *
   * \param I : The image used as background.
   * \param PExt1 : First extremity
   * \param PExt2 : Second extremity
   * \param A : Parameter a of the line equation a*i + b*j + c = 0
   * \param B : Parameter b of the line equation a*i + b*j + c = 0
   * \param C : Parameter c of the line equation a*i + b*j + c = 0
   * \param color : Color used to display the line.
   * \param thickness : Thickness of the line.
   */
  static void displayLine(const vpImage<vpRGBa> &I, const vpMeSite &PExt1, const vpMeSite &PExt2, const double &A,
                          const double &B, const double &C, const vpColor &color = vpColor::green,
                          unsigned int thickness = 1);

  /*!
   * Display of a moving line thanks to its equation parameters and its
   * extremities with all the site list.
   *
   * \param I : The image used as background.
   * \param PExt1 : First extremity
   * \param PExt2 : Second extremity
   * \param site_list : vpMeSite list
   * \param A : Parameter a of the line equation a*i + b*j + c = 0
   * \param B : Parameter b of the line equation a*i + b*j + c = 0
   * \param C : Parameter c of the line equation a*i + b*j + c = 0
   * \param color : Color used to display the line.
   * \param thickness : Thickness of the line.
   */
  static void displayLine(const vpImage<unsigned char> &I, const vpMeSite &PExt1, const vpMeSite &PExt2,
                          const std::list<vpMeSite> &site_list, const double &A, const double &B, const double &C,
                          const vpColor &color = vpColor::green, unsigned int thickness = 1);

  /*!
   * Display of a moving line thanks to its equation parameters and its
   * extremities with all the site list.
   *
   * \param I : The image used as background.
   * \param PExt1 : First extremity
   * \param PExt2 : Second extremity
   * \param site_list : vpMeSite list
   * \param A : Parameter a of the line equation a*i + b*j + c = 0
   * \param B : Parameter b of the line equation a*i + b*j + c = 0
   * \param C : Parameter c of the line equation a*i + b*j + c = 0
   * \param color : Color used to display the line.
   * \param thickness : Thickness of the line.
   */
  static void displayLine(const vpImage<vpRGBa> &I, const vpMeSite &PExt1, const vpMeSite &PExt2,
                          const std::list<vpMeSite> &site_list, const double &A, const double &B, const double &C,
                          const vpColor &color = vpColor::green, unsigned int thickness = 1);

  /*!
   * Computes the intersection point of two lines. The result is given in
   * the (i,j) frame.
   *
   * \param line1 : The first line.
   * \param line2 : The second line.
   * \param iP : The coordinates of the intersection point.
   *
   * \return Returns a boolean value which depends on the computation
   * success. True means that the computation ends successfully.
   */
  static bool intersection(const vpMeLine &line1, const vpMeLine &line2, vpImagePoint &iP);

  static void project(double a, double b, double c, const vpMeSite &P, vpImagePoint &iP);

protected:
  void computeDelta(double &delta, double i1, double j1, double i2, double j2);

  /*!
   * Compute the two parameters \f$(\rho, \theta)\f$ of the line.
   *
   * \param I : Image in which the line appears.
   */
  void computeRhoTheta(const vpImage<unsigned char> &I);

  /*!
   * Least squares method used to make the tracking more robust. It
   * ensures that the points taken into account to compute the right
   * equation belong to the line.
   *
   * \param I : Image in which the line appears.
   */
  void leastSquare(const vpImage<unsigned char> &I);

  void normalizeAngle(double &delta);

  /*!
   * Seek along the line defined by its equation to add points
   * in the detected holes. Useful in case of temporary occlusion
   *
   * \param I : Image in which the line appears.
   *
   * \return The function returns the number of points added to the list.
   *
   * \exception vpTrackingException::initializationError : Moving edges not
   * initialized.
   */
  unsigned int plugHoles(const vpImage<unsigned char> &I);

  /*!
   * Resample the line if the number of sample is less than 80% of the
   * expected value.
   *
   * \note The expected value is computed thanks to the length of the
   * line and the parameter which indicates the number of pixel between
   * two points (vpMe::sample_step).
   *
   * \param I : Image in which the line appears.
   */
  void reSample(const vpImage<unsigned char> &I);

  /*!
   * Construct a list of vpMeSite moving edges at a particular sampling
   * step between the two extremities of the line.
   *
   * \param I : Image in which the line appears.
   * \param doNotTrack : Inherited parameter, not used.
   *
   * \exception vpTrackingException::initializationError : Moving edges not
   * initialized.
   */
  virtual void sample(const vpImage<unsigned char> &I, bool doNotTrack = false) VP_OVERRIDE;

  /*!
   * Try to add points at both extremities of the line
   *
   * \param I : Image in which the line appears.
   *
   * \return The function returns the number of points added to the list.
   *
   * \exception vpTrackingException::initializationError : Moving edges not
   * initialized.
   */
  virtual unsigned int seekExtremities(const vpImage<unsigned char> &I);

  /*!
   * Seek in the list of good points its two extremities m_PExt[2]
   * These extremities are not strictly on the line
   *
   */
  void setExtremities();

  /*!
   * Set the alpha value of the different vpMeSite to the value of delta.
   */
  void updateDelta();

private:
  static void update_indices(double theta, int incr, int i, int j, int &i1, int &i2, int &j1, int &j2);

protected:
  vpMeSite m_PExt[2]; //!< Both extremities of good points in the list.
                      //!< These extremities are not strictly on the line

  double m_rho;     //!< rho parameter of the line
  double m_theta;   //!< theta parameter of the line
  double m_delta;   //!< Angle in rad between the extremities
  double m_delta_1; //!< Angle in rad between the extremities
  double m_angle;   //!< Angle in deg between the extremities
  double m_angle_1; //!< Angle in deg between the extremities
  int m_sign;       //!< Sign

  //! Flag to specify wether the intensity of the image at the middle point is
  //! used to compute the sign of rho or not.
  bool m_useIntensityForRho;

  double m_a; //!< Parameter a of the line equation a*i + b*j + c = 0
  double m_b; //!< Parameter b of the line equation a*i + b*j + c = 0
  double m_c; //!< Parameter c of the line equation a*i + b*j + c = 0

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
public:
  /*!
   * @name Deprecated functions
   */
  //@{

  /*!
   * \deprecated This method is deprecated. You should rather use setRhoSignFromIntensity().
   *
   * This method allows to turn off the computation of the sign of the rho
   * attribute based on the intensity near the middle point of the line. This
   * is usually done to distinguish between a black/white and a white/black
   * edge but it may be source of problem (ex. for a servoing example) when
   * this point can be occluded.
   *
   * \param useIntensityForRho : new value of the flag.
   */
  VP_DEPRECATED inline void computeRhoSignFromIntensity(bool useIntensityForRho)
  {
    m_useIntensityForRho = useIntensityForRho;
  }

  /*!
   * \deprecated Gets parameter a of the line equation a*i + b*j + c = 0
   * You should rather use get_ABC().
   */
  VP_DEPRECATED inline double getA() const { return m_a; }

  /*!
   * \deprecated Gets parameter b of the line equation a*i + b*j + c = 0
   * You should rather use get_ABC().
   */
  VP_DEPRECATED inline double getB() const { return m_b; }

  /*!
   * \deprecated Gets parameter c of the line equation a*i + b*j + c = 0
   * You should rather use get_ABC().
   */
  VP_DEPRECATED inline double getC() const { return m_c; }

  /*!
   * \deprecated Gets the equation parameters of the line
   * You should rather use get_ABC().
   */
  VP_DEPRECATED void getEquationParam(double &A, double &B, double &C) const
  {
    A = m_a;
    B = m_b;
    C = m_c;
  }

  /*!
   * \deprecated Use rather displayLine().
   */
  VP_DEPRECATED static void display(const vpImage<unsigned char> &I, const vpMeSite &PExt1, const vpMeSite &PExt2, const double &A,
                                    const double &B, const double &C, const vpColor &color = vpColor::green,
                                    unsigned int thickness = 1);

  /*!
   * \deprecated Use rather displayLine().
   */
  VP_DEPRECATED static void display(const vpImage<vpRGBa> &I, const vpMeSite &PExt1, const vpMeSite &PExt2, const double &A,
                                    const double &B, const double &C, const vpColor &color = vpColor::green,
                                    unsigned int thickness = 1);

  /*!
   * \deprecated Use rather displayLine().
   */
  VP_DEPRECATED static void display(const vpImage<unsigned char> &I, const vpMeSite &PExt1, const vpMeSite &PExt2,
                                    const std::list<vpMeSite> &site_list, const double &A, const double &B, const double &C,
                                    const vpColor &color = vpColor::green, unsigned int thickness = 1);

  /*!
   * \deprecated Use rather displayLine().
   */
  VP_DEPRECATED static void display(const vpImage<vpRGBa> &I, const vpMeSite &PExt1, const vpMeSite &PExt2,
                                    const std::list<vpMeSite> &site_list, const double &A, const double &B, const double &C,
                                    const vpColor &color = vpColor::green, unsigned int thickness = 1);
  //@}
#endif
};
END_VISP_NAMESPACE
#endif
